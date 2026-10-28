#!/usr/bin/env python3
"""Self-check for the task's verification step:
"Feed 20 test camera images with rotated/blurred ArUco tags; verify 0 false
gate triggers".

Run:  python3 test/test_gate_detection.py      (or: pytest test/)
No ROS needed. Writes the 20 generated images to test/images/ so you can
look at them and show them to the reviewer.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from aruco_lap_counter.lap_counter_node import (  # noqa: E402
    GateDetector, LapCounter, _get_dictionary)

GATES = [0, 1, 2, 3]
DICT_NAME = "DICT_4X4_50"
W, H = 640, 480
IMG_DIR = os.path.join(os.path.dirname(__file__), "images")


def marker_img(dictionary, marker_id, px=200):
    if hasattr(cv2.aruco, "generateImageMarker"):
        m = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    else:
        m = cv2.aruco.drawMarker(dictionary, marker_id, px)
    return cv2.copyMakeBorder(m, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)


def scene(marker_id, angle, blur_sigma, side_px, seed, dictionary):
    """Gray, noisy 'camera frame' with one rotated + blurred tag."""
    rng = np.random.default_rng(seed)
    bg = rng.integers(90, 150, (H, W), dtype=np.uint8)
    bg = cv2.GaussianBlur(bg, (0, 0), 3)
    if marker_id is not None:
        m = marker_img(dictionary, marker_id)
        scale = side_px / 200.0
        m = cv2.resize(m, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        h, w = m.shape
        M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
        big = int(np.hypot(h, w)) + 4
        M[0, 2] += (big - w) / 2
        M[1, 2] += (big - h) / 2
        rot = cv2.warpAffine(m, M, (big, big), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=255)
        x0, y0 = (W - big) // 2, (H - big) // 2
        bg[y0:y0 + big, x0:x0 + big] = rot
    if blur_sigma > 0:
        bg = cv2.GaussianBlur(bg, (0, 0), blur_sigma)
    noise = rng.normal(0, 4, bg.shape)
    bg = np.clip(bg.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return cv2.cvtColor(bg, cv2.COLOR_GRAY2BGR)


def build_dataset():
    d = _get_dictionary(DICT_NAME)
    cases = []  # (name, image, expected_trigger_id or None)
    # 12 real gate tags: all 4 gate IDs x 3 rotation/blur combinations
    combos = [(15, 0.8), (47, 1.5), (118, 2.0)]
    s = 0
    for gid in GATES:
        for ang, blur in combos:
            s += 1
            cases.append((f"gate{gid}_rot{ang}_blur{blur}",
                          scene(gid, ang, blur, 190, s, d), gid))
    # 3 decoy tags (valid ArUco but NOT our gates) -> must not trigger
    for did, ang, blur in [(17, 30, 1.0), (23, 75, 1.5), (42, 160, 2.0)]:
        s += 1
        cases.append((f"decoy{did}_rot{ang}_blur{blur}",
                      scene(did, ang, blur, 190, s, d), None))
    # 1 gate tag that is too far away (tiny) -> not "passing" yet
    s += 1
    cases.append(("gate1_too_far", scene(1, 20, 1.0, 28, s, d), None))
    # 4 negatives: blank, noise, plain black square, checkerboard
    s += 1
    cases.append(("neg_blank", scene(None, 0, 1.0, 0, s, d), None))
    rng = np.random.default_rng(99)
    noise = rng.integers(0, 255, (H, W, 3), dtype=np.uint8)
    cases.append(("neg_noise", cv2.GaussianBlur(noise, (0, 0), 1.2), None))
    sq = scene(None, 0, 0.0, 0, 7, d)
    cv2.rectangle(sq, (200, 120), (420, 340), (0, 0, 0), -1)
    cases.append(("neg_black_square", cv2.GaussianBlur(sq, (0, 0), 1.5), None))
    cb = np.indices((H, W)).sum(axis=0) // 40 % 2 * 255
    cb = cv2.cvtColor(cb.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cases.append(("neg_checkerboard", cv2.GaussianBlur(cb, (0, 0), 1.5), None))
    assert len(cases) == 20, len(cases)
    return cases


def test_20_images_zero_false_triggers():
    os.makedirs(IMG_DIR, exist_ok=True)
    det = GateDetector(GATES, DICT_NAME, min_side_px=40.0)
    false_triggers, misses, rows = 0, 0, []
    for name, img, expected in build_dataset():
        cv2.imwrite(os.path.join(IMG_DIR, name + ".png"), img)
        got = sorted(o.gate_id for o in det.detect(img))
        ok = (got == [] if expected is None else got == [expected])
        if expected is None and got:
            false_triggers += len(got)
        if expected is not None and got != [expected]:
            misses += 1
        rows.append((name, expected, got, "OK" if ok else "FAIL"))
    print(f"\n{'image':28s} {'expected':>8s} {'detected':>10s}")
    for r in rows:
        print(f"{r[0]:28s} {str(r[1]):>8s} {str(r[2]):>10s}  {r[3]}")
    print(f"\nFALSE TRIGGERS: {false_triggers}   MISSED REAL GATES: {misses}   (of 20 images)")
    assert false_triggers == 0, "false gate trigger(s) detected"
    assert misses == 0, "a real gate was missed"


def test_lap_sequence_logic():
    c = LapCounter(GATES, confirm_frames=3, cooldown_s=2.0)
    t = 0.0

    def see(ids, frames=3, dt=0.1):
        nonlocal t
        ev = None
        for _ in range(frames):
            t += dt
            ev = c.update(ids, t) or ev
        t += 3.0  # rover drives away; clears cooldown
        c.update([], t)
        return ev

    assert see([2]) is None and c.laps == 0          # before start: ignored
    assert see([0]) is not None                       # START
    assert see([2]) is None                           # out of order: ignored
    assert see([1]) and see([2]) and see([3])         # in order
    assert c.laps == 0
    assert see([0]) and c.laps == 1                   # lap 1
    assert see([0]) is None and c.laps == 1           # re-seeing start: no double count
    assert see([1]) and see([2]) and see([3]) and see([0]) and c.laps == 2
    # a 1-frame flicker must never fire
    c2 = LapCounter(GATES, confirm_frames=3)
    assert c2.update([0], 0.0) is None and c2.update([], 0.1) is None
    assert c2.update([0], 0.2) is None and not c2.started


if __name__ == "__main__":
    test_20_images_zero_false_triggers()
    test_lap_sequence_logic()
    print("lap sequence logic: OK\nALL CHECKS PASSED")
