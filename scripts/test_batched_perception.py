#!/usr/bin/env python3
"""
Unit & Integration Test for Batched SwarmPerceptionNode (Phase 2 Remediation - Stage 2)

Verifies:
  1. Node initializes with configurable batch size and GPU device
  2. Multi-camera image callbacks update frame buffers without blocking
  3. Single-pass batched inference processes multi-drone frames
  4. Frame uniqueness is enforced (no frame processed twice)
  5. Simulation timestamps are preserved in published detection JSON
  6. Output detection topics /drone_{i}/perception/detections receive valid messages
"""

import os
import sys
import time
import json
import numpy as np

# Ensure workspace packages can be imported
ws_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/swarm_controller'))
if ws_src not in sys.path:
    sys.path.insert(0, ws_src)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from swarm_controller.swarm_perception_node import SwarmPerceptionNode


def run_test(name: str, fn):
    try:
        fn()
        print(f"[PASS] {name}")
        return True
    except AssertionError as e:
        print(f"[FAIL] {name}: AssertionError: {e}")
        return False
    except Exception as e:
        print(f"[FAIL] {name}: Exception: {e}")
        return False


def test_batched_perception_execution():
    rclpy.init()
    node = SwarmPerceptionNode()
    bridge = CvBridge()

    # Track published detections
    received_detections = {i: [] for i in range(6)}

    def make_detection_listener(d_id):
        def cb(msg):
            try:
                data = json.loads(msg.data)
                received_detections[d_id].append(data)
            except Exception as e:
                print(f"Error parsing detection: {e}")
        return cb

    sub_node = Node('test_perception_listener')
    subs = []
    for i in range(6):
        s = sub_node.create_subscription(
            String,
            f'/drone_{i}/perception/detections',
            make_detection_listener(i),
            10
        )
        subs.append(s)

    # 1. Generate synthetic 640x480 test frames for all 6 drones
    test_ts = 123.456
    for i in range(6):
        # Create dummy image
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Put white box in center
        img[200:280, 280:360] = 255
        ros_img = bridge.cv2_to_imgmsg(img, 'bgr8')
        ros_img.header.stamp.sec = int(test_ts)
        ros_img.header.stamp.nanosec = int((test_ts - int(test_ts)) * 1e9)
        ros_img.header.frame_id = f"camera_link_drone_{i}"

        # Inject into perception node callback directly
        node.image_callback(ros_img, i)

    # Verify all 6 buffers are filled with un-inferred frames
    with node.buffer_lock:
        for i in range(6):
            rec = node.frame_buffers[i]
            assert rec is not None, f"Drone {i} buffer must be populated"
            assert rec['inferred'] is False, f"Drone {i} frame must not be marked inferred before processing"
            assert np.isclose(rec['sim_ts'], test_ts), f"Drone {i} timestamp must match {test_ts}"

    # 2. Trigger batched processing
    node.process_batch()

    # Spin sub_node once to receive published detection messages
    start_wait = time.time()
    while time.time() - start_wait < 1.0:
        rclpy.spin_once(sub_node, timeout_sec=0.05)
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(len(received_detections[i]) >= 1 for i in range(6)):
            break

    # 3. Assertions
    for i in range(6):
        # Detections received
        assert len(received_detections[i]) >= 1, f"Drone {i} must receive detection message"
        det_data = received_detections[i][0]
        assert det_data['drone_id'] == i, f"Expected drone_id={i}, got {det_data['drone_id']}"
        assert np.isclose(det_data['timestamp'], test_ts, atol=1e-3), (
            f"Simulation timestamp not preserved! Expected {test_ts}, got {det_data['timestamp']}"
        )
        assert det_data['image_width'] == 640
        assert det_data['image_height'] == 480
        assert isinstance(det_data['detections'], list)

    # 4. Verify no frame is inferred twice
    # Calling process_batch again immediately when no new frames arrived must do NOTHING
    prev_infer_count = node.total_frames_inferred
    node.process_batch()
    assert node.total_frames_inferred == prev_infer_count, (
        "Inferred count changed! Frames must not be re-inferred when no new image arrived."
    )

    node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()


def main():
    print("=" * 75)
    print("PHASE 2 REMEDIATION: STAGE 2 BATCHED PERCEPTION NODE TESTS")
    print("=" * 75)

    tests = [
        ("Batched Multi-Camera Perception & Timestamp Preservation", test_batched_perception_execution)
    ]

    passed = 0
    for name, fn in tests:
        if run_test(name, fn):
            passed += 1

    print("=" * 75)
    print(f"RESULTS: {passed}/{len(tests)} TESTS PASSED")
    print("=" * 75)
    return 0 if passed == len(tests) else 1


if __name__ == '__main__':
    sys.exit(main())
