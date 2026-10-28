# aruco_lap_counter  (SEDS-TASK-ROV-SRILANKA-004)

ROS 2 Python node that detects ArUco gate markers from a camera topic and counts laps.

## How it works
1. `GateDetector` finds ArUco tags (OpenCV) and keeps only tags that
   - have an ID in `gate_sequence` (decoys / misreads are dropped),
   - are large enough (`min_side_px`) to count as "at the gate",
   - are not wildly skewed.
2. `LapCounter` is an ordered state machine. Gate 0 is start/finish.
   A lap = 0 -> 1 -> 2 -> 3 -> 0 in order. Wrong-order sightings are ignored.
   A gate must be seen in `confirm_frames` consecutive frames, with a `cooldown_s`
   between repeats, so single-frame flicker cannot trigger anything.

## Run the self-check (no ROS needed)
    pip install opencv-contrib-python numpy
    python3 test/test_gate_detection.py
Generates 20 rotated/blurred test images in `test/images/` and asserts
0 false triggers and 0 missed gates, plus a lap-sequence test.

## Run in ROS 2 Humble
    cp -r aruco_lap_counter ~/ros2_ws/src/
    cd ~/ros2_ws && colcon build --packages-select aruco_lap_counter
    source install/setup.bash
    ros2 launch aruco_lap_counter lap_counter.launch.py image_topic:=/camera/image_raw
    ros2 topic echo /lap_count

## Parameters
| name | default | meaning |
|---|---|---|
| image_topic | /camera/image_raw | camera input |
| gate_sequence | [0,1,2,3] | gate IDs in lap order, first = start/finish |
| aruco_dictionary | DICT_4X4_50 | must match the printed tags |
| min_side_px | 40.0 | min tag size to count as a gate pass |
| confirm_frames | 3 | consecutive frames required |
| cooldown_s | 2.0 | min seconds between triggers of the same gate |
| publish_debug_image | false | publish annotated image on gate_debug_image |
