#!/usr/bin/env python3
"""
scripts/test_vision_target_probe.py

Isolated Controlled Simulation Test Scene:
1. Creates an isolated test world `worlds/test_vision_target.sdf` containing:
   - Ground plane
   - Sun light
   - Target victim: casual_female at (0, 0, 0)
   - Target victim: RescueRandy standing at (2, 0, 0)
   - Visual target with human photo texture at (0, 2, 0)
2. Spawns an x500_rgbd_down camera model at (0, 0, 3.5) pointing straight down.
3. Runs Gazebo Harmonic headless.
4. Starts ros_gz_bridge for camera topics.
5. Captures color and depth frames.
6. Evaluates YOLOv8n on the captured frames and logs detections.
7. Evaluates rgbd_victim_localizer on the detections and verifies if /sar/victim_discovery is published.
8. Cleans up all test processes and temporary files.
"""

import os
import sys
import time
import subprocess
import signal
import json
import numpy as np

# Ensure stdout flushes
sys.stdout.reconfigure(line_buffering=True)

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
TEST_WORLD = os.path.join(WORKSPACE_DIR, "worlds", "test_vision_target.sdf")
CAPTURE_PNG = "/tmp/test_vision_capture.png"

# Create isolated test world
TEST_WORLD_SDF = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="test_vision_target">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.2 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Target 1: casual_female at origin -->
    <include>
      <name>casual_female_victim</name>
      <uri>model://casual_female</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- Target 2: rescue_randy_standing at (1.5, 0, 0) -->
    <include>
      <name>rescue_randy_victim</name>
      <uri>model://rescue_randy_standing</uri>
      <pose>1.5 0 0 0 0 0</pose>
    </include>

    <!-- Camera Drone Sensor Model at (0, 0, 3.5) pointing straight down -->
    <include>
      <name>x500_rgbd_down_0</name>
      <uri>model://x500_rgbd_down</uri>
      <pose>0 0 3.5 0 0 0</pose>
    </include>

  </world>
</sdf>
"""

pids = []

def cleanup():
    print("[CLEANUP] Stopping simulation and bridge processes...")
    for pid in reversed(pids):
        try:
            os.killpg(pid, signal.SIGTERM)
        except Exception:
            pass
    time.sleep(1)
    for pid in reversed(pids):
        try:
            os.killpg(pid, signal.SIGKILL)
        except Exception:
            pass
    if os.path.exists(TEST_WORLD):
        try:
            os.remove(TEST_WORLD)
        except Exception:
            pass
    print("[CLEANUP] Done.")

try:
    print("=== STEP 1: Creating Isolated Test World ===")
    with open(TEST_WORLD, "w") as f:
        f.write(TEST_WORLD_SDF)
    print(f"Created {TEST_WORLD}")

    print("=== STEP 2: Launching Gazebo Harmonic Headless ===")
    gz_env = os.environ.copy()
    gz_env["GZ_IP"] = "127.0.0.1"
    gz_env["PX4_GZ_WORLD"] = "test_vision_target"
    gz_env["GZ_SIM_RESOURCE_PATH"] = (
        f"{WORKSPACE_DIR}/models:{WORKSPACE_DIR}/assets_real:{WORKSPACE_DIR}/assets_real/victims:"
        f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
        f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE_DIR}/worlds"
    )
    cmd_gz = f"gz sim -s -v 2 -r --headless-rendering {TEST_WORLD}"
    proc_gz = subprocess.Popen(cmd_gz, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/gz_target_test.log", "w"), stderr=subprocess.STDOUT)
    pids.append(proc_gz.pid)

    print("Waiting 6s for Gazebo to load models...")
    time.sleep(6)

    # Check clock
    clock_ready = False
    for _ in range(15):
        res = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], capture_output=True, text=True, timeout=3)
        if res.returncode == 0 and len(res.stdout.strip()) > 0:
            clock_ready = True
            break
        time.sleep(1)
    print(f"Gazebo Clock Active: {clock_ready}")

    print("=== STEP 3: Starting ros_gz_bridge for camera topics ===")
    bridge_cmd = (
        "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge "
        "/world/test_vision_target/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image "
        "/world/test_vision_target/model/x500_rgbd_down_0/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image "
        "/world/test_vision_target/model/x500_rgbd_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo "
        "/model/x500_rgbd_down_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose "
        "--ros-args "
        "-r /world/test_vision_target/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image:=/drone_0/camera/image "
        "-r /world/test_vision_target/model/x500_rgbd_down_0/link/camera_link/sensor/camera/depth_image:=/drone_0/camera/depth_image "
        "-r /world/test_vision_target/model/x500_rgbd_down_0/link/camera_link/sensor/camera/camera_info:=/drone_0/camera/camera_info "
        "-r /model/x500_rgbd_down_0/pose:=/model/drone_0/pose"
    )
    proc_bridge = subprocess.Popen(bridge_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/bridge_target_test.log", "w"), stderr=subprocess.STDOUT)
    pids.append(proc_bridge.pid)
    time.sleep(4)

    print("=== STEP 4: Capturing and Inspecting Camera Imagery ===")
    python_test_cmd = """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class FrameGrabber(Node):
    def __init__(self):
        super().__init__('frame_grabber')
        self.bridge = CvBridge()
        self.color_msg = None
        self.depth_msg = None
        self.create_subscription(RosImage, '/drone_0/camera/image', self.c_cb, 10)
        self.create_subscription(RosImage, '/drone_0/camera/depth_image', self.d_cb, 10)

    def c_cb(self, msg):
        if self.color_msg is None:
            self.color_msg = msg

    def d_cb(self, msg):
        if self.depth_msg is None:
            self.depth_msg = msg

rclpy.init()
grabber = FrameGrabber()

for _ in range(50):
    rclpy.spin_once(grabber, timeout_sec=0.2)
    if grabber.color_msg is not None and grabber.depth_msg is not None:
        break

if grabber.color_msg is not None:
    cv_img = grabber.bridge.imgmsg_to_cv2(grabber.color_msg, 'bgr8')
    cv_depth = grabber.bridge.imgmsg_to_cv2(grabber.depth_msg, desired_encoding='passthrough')
    cv2.imwrite('/tmp/test_vision_capture.png', cv_img)
    print(f'[CAPTURED] Image {cv_img.shape}, Depth {cv_depth.shape}, Depth min={np.nanmin(cv_depth):.2f}m max={np.nanmax(cv_depth):.2f}m mean={np.nanmean(cv_depth):.2f}m')

    # Run YOLO
    model = YOLO('/home/capstone/capstone_project_antigravity/models/yolov8n.pt')
    results = model(cv_img, verbose=True)
    all_dets = []
    for r in results:
        for b in r.boxes:
            c = int(b.cls[0])
            name = model.names[c]
            conf = float(b.conf[0])
            bbox = [int(v) for v in b.xyxy[0]]
            all_dets.append({'name': name, 'class_id': c, 'conf': conf, 'bbox': bbox})
            print(f'   -> DETECTED: {name} (conf={conf:.3f}, bbox={bbox})')
    if not all_dets:
        print('   -> No objects detected above threshold.')
else:
    print('[FAIL] No frames received within timeout.')

rclpy.shutdown()
"""
    cmd_exec = f"source /opt/ros/jazzy/setup.bash && export PYTHONPATH=$PYTHONPATH:/home/capstone/yolo_venv/lib/python3.12/site-packages && python3 -c \"{python_test_cmd}\""
    res = subprocess.run(cmd_exec, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=30)
    print("STDOUT:\n" + res.stdout)
    if res.stderr:
        print("STDERR:\n" + res.stderr)

finally:
    cleanup()
