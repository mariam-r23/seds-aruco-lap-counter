#!/usr/bin/env python3
"""Fake camera: publishes images from a folder on /camera/image_raw.

Use this when you have no real camera. The lap counter node cannot tell the
difference, so it runs exactly as it would on the rover.

    ros2 run aruco_lap_counter image_folder_publisher
    ros2 run aruco_lap_counter image_folder_publisher --ros-args -p folder:=/path/to/images

If the folder does not exist or is empty, a demo lap is generated for you:
gate 0 -> 1 -> 2 -> 3 -> 0 (rotated + blurred tags), i.e. exactly 1 lap.
Images are played in filename order, so name your own files 01_..., 02_...
"""
import glob
import os
from typing import List

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# ROS-free helpers (also used by the offline check below)
# --------------------------------------------------------------------------- #
def _tag_scene(dictionary, marker_id, angle, blur, side_px=190, seed=0, w=640, h=480):
    rng = np.random.default_rng(seed)
    bg = cv2.GaussianBlur(rng.integers(90, 150, (h, w), dtype=np.uint8), (0, 0), 3)
    if hasattr(cv2.aruco, "generateImageMarker"):
        m = cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
    else:
        m = cv2.aruco.drawMarker(dictionary, marker_id, 200)
    m = cv2.copyMakeBorder(m, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)
    m = cv2.resize(m, None, fx=side_px / 200.0, fy=side_px / 200.0,
                   interpolation=cv2.INTER_AREA)
    mh, mw = m.shape
    big = int(np.hypot(mh, mw)) + 4
    M = cv2.getRotationMatrix2D((mw / 2, mh / 2), angle, 1.0)
    M[0, 2] += (big - mw) / 2
    M[1, 2] += (big - mh) / 2
    rot = cv2.warpAffine(m, M, (big, big), borderValue=255)
    y0, x0 = (h - big) // 2, (w - big) // 2
    bg[y0:y0 + big, x0:x0 + big] = rot
    bg = cv2.GaussianBlur(bg, (0, 0), blur)
    return cv2.cvtColor(bg, cv2.COLOR_GRAY2BGR)


def generate_demo_images(folder: str, dictionary_name: str = "DICT_4X4_50"):
    """Write 5 images (gates 0,1,2,3,0) into ``folder``."""
    from aruco_lap_counter.lap_counter_node import _get_dictionary
    d = _get_dictionary(dictionary_name)
    os.makedirs(folder, exist_ok=True)
    plan = [(0, 15, 0.8), (1, 47, 1.5), (2, 118, 2.0), (3, 30, 1.0), (0, 75, 1.5)]
    for n, (gid, ang, blur) in enumerate(plan, start=1):
        img = _tag_scene(d, gid, ang, blur, seed=n)
        cv2.imwrite(os.path.join(folder, f"{n:02d}_gate{gid}.png"), img)


def load_images(folder: str) -> List[np.ndarray]:
    paths = []
    for ext in ("png", "jpg", "jpeg", "bmp"):
        paths += glob.glob(os.path.join(folder, f"*.{ext}"))
    paths.sort()
    imgs = [cv2.imread(p) for p in paths]
    return [i for i in imgs if i is not None]


def build_schedule(images: List[np.ndarray], repeat: int = 5, gap: int = 4):
    """Each image is shown ``repeat`` frames (the rover 'at the gate'),
    followed by ``gap`` blank frames (the rover driving to the next gate)."""
    frames = []
    for img in images:
        frames += [img] * repeat
        frames += [np.full_like(img, 110)] * gap
    return frames


# --------------------------------------------------------------------------- #
# ROS 2 node
# --------------------------------------------------------------------------- #
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class ImageFolderPublisher(Node):
        def __init__(self):
            super().__init__("image_folder_publisher")
            self.declare_parameter("folder", os.path.expanduser("~/aruco_demo_images"))
            self.declare_parameter("topic", "/camera/image_raw")
            self.declare_parameter("rate_hz", 5.0)
            self.declare_parameter("repeat", 5)
            self.declare_parameter("gap", 4)
            self.declare_parameter("loop", False)
            self.declare_parameter("aruco_dictionary", "DICT_4X4_50")

            folder = self.get_parameter("folder").value
            images = load_images(folder) if os.path.isdir(folder) else []
            if not images:
                self.get_logger().info(f"No images in {folder}; generating a demo lap there.")
                generate_demo_images(folder, self.get_parameter("aruco_dictionary").value)
                images = load_images(folder)
            self.frames = build_schedule(
                images, self.get_parameter("repeat").value, self.get_parameter("gap").value)
            self.loop = bool(self.get_parameter("loop").value)
            self.idx = 0
            self.bridge = CvBridge()
            self.pub = self.create_publisher(Image, self.get_parameter("topic").value, 5)
            self.timer = self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)
            self.get_logger().info(
                f"Publishing {len(images)} images ({len(self.frames)} frames) from {folder}")

        def tick(self):
            if self.idx >= len(self.frames):
                if self.loop:
                    self.idx = 0
                else:
                    self.get_logger().info("Done publishing all images.")
                    self.timer.cancel()
                    return
            msg = self.bridge.cv2_to_imgmsg(self.frames[self.idx], "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.pub.publish(msg)
            self.idx += 1

    rclpy.init(args=args)
    node = ImageFolderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
