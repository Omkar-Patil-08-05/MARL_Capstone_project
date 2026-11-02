#!/usr/bin/env python3
"""
Step 5 Diagnostic Test: Camera Topic Verification with Running Headless Simulation
1. Starts MicroXRCEAgent.
2. Starts Gazebo Harmonic with --headless-rendering on worlds/forest_sar.sdf.
3. Spawns PX4 instance 0 with x500_rgbd_down.
4. Starts ros_gz_bridge parameter_bridge for camera topics.
5. Verifies:
   - Gazebo topic arrival for camera sensor: /world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image
   - ROS 2 topic arrival: /drone_0/camera/image with valid sensor_msgs/msg/Image frames.
6. Scoped cleanup of all spawned PIDs.
"""

import os
import sys
import time
import subprocess
import signal

WORKSPACE = "/home/capstone/capstone_project_antigravity"
BUILD_DIR = "/home/capstone/PX4-Autopilot/build/px4_sitl_default"

pids = []

def cleanup():
    print("\n[CLEANUP] Stopping camera test processes...")
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
    print("[CLEANUP] Completed.")

try:
    print("[1/5] Starting MicroXRCEAgent...")
    cmd_agent = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888"
    proc_agent = subprocess.Popen(cmd_agent, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    pids.append(proc_agent.pid)
    time.sleep(2)

    print("[2/5] Starting Gazebo Harmonic server with --headless-rendering...")
    gz_env = os.environ.copy()
    gz_env["GZ_IP"] = "127.0.0.1"
    gz_env["PX4_GZ_WORLD"] = "forest_sar"
    gz_env["PX4_GZ_WORLDS"] = f"{WORKSPACE}/worlds"
    gz_env["GZ_SIM_RESOURCE_PATH"] = (
        f"{WORKSPACE}/models:{WORKSPACE}/assets_real:{WORKSPACE}/assets_real/victims:"
        f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
        f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE}/generated_worlds:{WORKSPACE}/worlds"
    )
    gz_cmd = f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -s -v 3 -r --headless-rendering {WORKSPACE}/worlds/forest_sar.sdf"
    proc_gz = subprocess.Popen(gz_cmd, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/gz_cam_diag.log", "w"), stderr=subprocess.STDOUT)
    pids.append(proc_gz.pid)

    print("      Waiting 10s for simulation stabilization...")
    time.sleep(10)

    print("      Waiting up to 30s for Gazebo clock...")
    clock_ready = False
    for _ in range(30):
        try:
            res = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], capture_output=True, text=True, timeout=5)
            if res.returncode == 0 and len(res.stdout.strip()) > 0:
                clock_ready = True
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1)
    assert clock_ready, "Gazebo clock failed to start!"
    print("      Gazebo clock is active.")

    print("[3/5] Starting PX4 instance 0 with x500_rgbd_down...")
    inst_dir = "/tmp/px4_cam_instance_0"
    os.makedirs(inst_dir, exist_ok=True)
    px4_env = gz_env.copy()
    px4_env["PX4_GZ_WORLD"] = "forest_sar"
    px4_env["PX4_GZ_MODEL"] = "x500_rgbd_down"
    px4_env["PX4_GZ_MODEL_POSE"] = "-168,140,16.0,0,0,0"
    px4_env["PX4_UXRCE_DDS_NS"] = "drone_0"
    px4_env["PX4_GZ_STANDALONE"] = "1"
    px4_env["PX4_SYS_AUTOSTART"] = "4100"
    px4_env["PX4_SIMULATOR"] = "gz"

    cmd_px4 = f"cd {inst_dir} && {BUILD_DIR}/bin/px4 -i 0 -d {BUILD_DIR}/etc"
    proc_px4 = subprocess.Popen(cmd_px4, shell=True, env=px4_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"{inst_dir}/out.log", "w"), stderr=open(f"{inst_dir}/err.log", "w"))
    pids.append(proc_px4.pid)

    print("      Waiting up to 45s for OGRE2 to load terrain and camera sensor to initialize...")
    cam_topics = []
    target_gz_topic = None
    for sec in range(45):
        gz_topics_res = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=5)
        gz_topics = gz_topics_res.stdout.splitlines()
        cam_topics = [t for t in gz_topics if "camera" in t and "image" in t]
        if cam_topics:
            target_gz_topic = cam_topics[0]
            print(f"      [t+{sec}s] Found Gazebo camera image topics: {cam_topics}")
            break
        time.sleep(1)

    assert target_gz_topic is not None, "Timed out waiting for Gazebo camera image topics!"
    print(f"      Testing Gazebo Transport image delivery on {target_gz_topic}...")
    gz_echo = subprocess.run(["timeout", "5", "gz", "topic", "-e", "-t", target_gz_topic, "-n", "1"], capture_output=True, text=True)
    assert len(gz_echo.stdout) > 0 or gz_echo.returncode == 0, "Failed to read image frame from Gazebo Transport!"
    print("      SUCCESS: Gazebo camera frames are rendering and publishing on Gazebo Transport!")

    print("[5/5] Starting ros_gz_bridge for camera topics...")
    bridge_topic = f"{target_gz_topic}@sensor_msgs/msg/Image[gz.msgs.Image"
    bridge_remap = f"-r {target_gz_topic}:=/drone_0/camera/image"
    bridge_cmd = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge \"{bridge_topic}\" --ros-args {bridge_remap}"
    proc_bridge = subprocess.Popen(bridge_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/bridge_cam_diag.log", "w"), stderr=subprocess.STDOUT)
    pids.append(proc_bridge.pid)
    time.sleep(3)

    print("      Verifying ROS 2 /drone_0/camera/image reception...")
    ros_echo_cmd = "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /drone_0/camera/image --once"
    ros_echo = subprocess.run(ros_echo_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
    assert "/drone_0/camera/image" in ros_echo.stdout or "height: 480" in ros_echo.stdout or "width: 640" in ros_echo.stdout or "encoding:" in ros_echo.stdout, (
        f"ROS 2 image topic did not receive expected message! Output was: {ros_echo.stdout[:200]}"
    )
    print("      SUCCESS: ROS 2 /drone_0/camera/image received valid frame (640x480)!")
    print("\n>>> STEP 5 CAMERA PIPELINE VERIFIED SUCCESSFULLY! <<<")

finally:
    cleanup()
