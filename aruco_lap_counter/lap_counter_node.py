#!/usr/bin/env python3
"""ArUco marker lap counter & gate detection (SEDS-TASK-ROV-SRILANKA-004).

Structure
---------
* ``GateDetector``   - OpenCV-only ArUco detection + false-trigger filtering.
* ``LapCounter``     - pure-Python state machine (ordered gates -> laps).
* ``LapCounterNode`` - thin ROS 2 wrapper (rclpy) around the two classes.

The first two classes do not import ROS, so they can be unit-tested on any
laptop that has OpenCV (with the aruco module) and numpy.

Works with OpenCV >= 4.7 (ArucoDetector API) and with the OpenCV 4.5.x that
ships with ROS 2 Humble / Ubuntu 22.04 (legacy detectMarkers API).

Topics (defaults, all remappable)
    sub  /camera/image_raw      sensor_msgs/Image
    pub  lap_count              std_msgs/Int32   (completed laps)
    pub  gate_event             std_msgs/String  ("gate=2 PASSED lap=0 next=3")
    pub  gate_debug_image       sensor_msgs/Image (annotated, if enabled)
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# OpenCV version shim
# --------------------------------------------------------------------------- #
def _get_dictionary(name: str):
    """Return a cv2.aruco dictionary from a name like 'DICT_4X4_50'."""
    aruco = cv2.aruco
    if not hasattr(aruco, name):
        raise ValueError(f"Unknown ArUco dictionary '{name}'")
    dict_id = getattr(aruco, name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)  # very old builds


def _make_params():
    aruco = cv2.aruco
    if hasattr(aruco, "DetectorParameters"):
        p = aruco.DetectorParameters()
    else:
        p = aruco.DetectorParameters_create()
    # Tuned for blurred / rotated tags.
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 6
    p.minMarkerPerimeterRate = 0.02
    p.polygonalApproxAccuracyRate = 0.05
    p.errorCorrectionRate = 0.3  # lower than default 0.6 -> fewer false IDs
    p.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return p


# --------------------------------------------------------------------------- #
# Detection
# --------------------------------------------------------------------------- #
@dataclass
class GateObservation:
    gate_id: int
    side_px: float  # mean side length of the marker in pixels
    corners: np.ndarray = field(repr=False, default=None)


class GateDetector:
    """Detects ArUco tags and keeps only those that are real, close gates."""

    def __init__(self,
                 gate_ids: Sequence[int],
                 dictionary: str = "DICT_4X4_50",
                 min_side_px: float = 40.0,
                 max_skew: float = 0.5):
        self.gate_ids = set(int(i) for i in gate_ids)
        self.min_side_px = float(min_side_px)
        self.max_skew = float(max_skew)
        self._dict = _get_dictionary(dictionary)
        self._params = _make_params()
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(self._dict, self._params)
        else:
            self._detector = None

    def _detect_raw(self, gray):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params)
        return corners, ids

    def detect(self, frame: np.ndarray) -> List[GateObservation]:
        """Return observations that pass every false-trigger filter."""
        if frame is None or frame.size == 0:
            return []
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        corners, ids = self._detect_raw(gray)
        if ids is None:
            return []

        out: List[GateObservation] = []
        for c, i in zip(corners, ids.flatten()):
            gid = int(i)
            # Filter 1: ID must be one of OUR gates (rejects decoys/misreads).
            if gid not in self.gate_ids:
                continue
            pts = c.reshape(4, 2).astype(np.float32)
            sides = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
            side = float(sides.mean())
            # Filter 2: must be near enough to count as "passing the gate".
            if side < self.min_side_px:
                continue
            # Filter 3: reject extreme perspective skew (likely a misdetection).
            if (sides.max() - sides.min()) / max(float(sides.max()), 1e-6) > self.max_skew:
                continue
            out.append(GateObservation(gid, side, pts))
        return out


# --------------------------------------------------------------------------- #
# Lap state machine (no OpenCV / ROS needed)
# --------------------------------------------------------------------------- #
class LapCounter:
    """Counts laps from gate sightings.

    ``gate_sequence`` is the ordered list of gate IDs for one lap; the first
    entry is the start/finish gate, e.g. [0, 1, 2, 3].  A lap is complete when
    gates 1, 2, 3 were passed IN ORDER and the start/finish gate (0) is seen
    again.  Out-of-order, repeated or unknown sightings never change the count.

    A gate counts as "passed" only after ``confirm_frames`` consecutive frames
    containing it, and the same gate cannot fire again within ``cooldown_s``.
    """

    def __init__(self, gate_sequence: Sequence[int],
                 confirm_frames: int = 3, cooldown_s: float = 2.0):
        if len(gate_sequence) < 2:
            raise ValueError("gate_sequence needs at least 2 gates")
        self.sequence = [int(g) for g in gate_sequence]
        self.confirm_frames = max(1, int(confirm_frames))
        self.cooldown_s = float(cooldown_s)
        self.reset()

    def reset(self):
        self.laps = 0
        self.started = False
        self._next_idx = 1  # index into sequence of the gate we wait for
        self._streak: Dict[int, int] = {}
        self._last_fire: Dict[int, float] = {}

    @property
    def next_gate(self) -> int:
        return self.sequence[self._next_idx % len(self.sequence)]

    def update(self, seen_ids: Sequence[int], t: float) -> Optional[str]:
        """Feed the gate IDs seen in ONE frame. Returns an event string or None."""
        seen = set(int(i) for i in seen_ids)
        for gid in list(self._streak):
            if gid not in seen:
                self._streak[gid] = 0
        for gid in seen:
            self._streak[gid] = self._streak.get(gid, 0) + 1

        event = None
        for gid in sorted(seen):
            if self._streak[gid] != self.confirm_frames:
                continue  # not confirmed yet, or already fired for this sighting
            if t - self._last_fire.get(gid, -1e9) < self.cooldown_s:
                continue  # cooldown
            e = self._on_gate_passed(gid)
            if e:
                self._last_fire[gid] = t
                event = e
        return event

    def _on_gate_passed(self, gid: int) -> Optional[str]:
        start = self.sequence[0]
        if not self.started:
            if gid == start:
                self.started = True
                self._next_idx = 1
                return f"gate={gid} START lap={self.laps} next={self.next_gate}"
            return None
        if gid != self.next_gate:
            return None  # wrong order -> ignore
        if gid == start:  # closed the loop
            self.laps += 1
            self._next_idx = 1
            return f"gate={gid} LAP_COMPLETE lap={self.laps} next={self.next_gate}"
        self._next_idx += 1
        return f"gate={gid} PASSED lap={self.laps} next={self.next_gate}"


# --------------------------------------------------------------------------- #
# ROS 2 node
# --------------------------------------------------------------------------- #
def _build_node_class():
    """Import ROS lazily so the logic above is importable without ROS."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Int32, String
    from cv_bridge import CvBridge

    class LapCounterNode(Node):
        def __init__(self):
            super().__init__("aruco_lap_counter")
            self.declare_parameter("image_topic", "/camera/image_raw")
            self.declare_parameter("gate_sequence", [0, 1, 2, 3])
            self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
            self.declare_parameter("min_side_px", 40.0)
            self.declare_parameter("confirm_frames", 3)
            self.declare_parameter("cooldown_s", 2.0)
            self.declare_parameter("publish_debug_image", False)

            seq = [int(g) for g in self.get_parameter("gate_sequence").value]
            self.detector = GateDetector(
                seq,
                self.get_parameter("aruco_dictionary").value,
                self.get_parameter("min_side_px").value)
            self.counter = LapCounter(
                seq,
                self.get_parameter("confirm_frames").value,
                self.get_parameter("cooldown_s").value)
            self.debug = bool(self.get_parameter("publish_debug_image").value)
            self.bridge = CvBridge()

            self.lap_pub = self.create_publisher(Int32, "lap_count", 10)
            self.evt_pub = self.create_publisher(String, "gate_event", 10)
            self.dbg_pub = None
            if self.debug:
                self.dbg_pub = self.create_publisher(Image, "gate_debug_image", 1)
            self.create_subscription(
                Image, self.get_parameter("image_topic").value, self.on_image, 5)
            self.get_logger().info(
                f"Lap counter ready. Gate order: {seq}, waiting for gate {seq[0]}")

        def on_image(self, msg):
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            obs = self.detector.detect(frame)
            t = self.get_clock().now().nanoseconds * 1e-9
            event = self.counter.update([o.gate_id for o in obs], t)
            if event:
                self.evt_pub.publish(String(data=event))
                self.lap_pub.publish(Int32(data=self.counter.laps))
                self.get_logger().info(event)
            if self.dbg_pub is not None:
                for o in obs:
                    cv2.polylines(frame, [o.corners.astype(int)], True, (0, 255, 0), 2)
                    cv2.putText(frame, f"gate {o.gate_id}",
                                tuple(int(v) for v in o.corners[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    return LapCounterNode, rclpy


def main(args=None):
    LapCounterNode, rclpy = _build_node_class()
    rclpy.init(args=args)
    node = LapCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
