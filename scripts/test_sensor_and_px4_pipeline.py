#!/usr/bin/env python3
"""
Diagnostic test script to verify PX4 sensors and estimator readiness on forest_sar:
1. Launches MicroXRCEAgent
2. Launches Gazebo Harmonic with forest_sar.sdf
3. Launches PX4 SITL instance 0 with x500_rgbd_down
4. Monitors Gazebo topics and ROS 2 topics:
   - IMU, Baro, Mag, GPS (Gazebo Transport)
   - /drone_0/fmu/out/sensor_combined (ROS 2)
   - /drone_0/fmu/out/vehicle_odometry (ROS 2)
   - /drone_0/fmu/out/vehicle_status (ROS 2)
5. Verifies absence of preflight failures (Accel, Gyro, Baro, Compass, EKF2).
6. Scoped cleanup of all spawned PIDs.
"""
import os
import sys
import time
import subprocess
import signal

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
BUILD_DIR = "/home/capstone/PX4-Autopilot/build/px4_sitl_default"

pids = []

def cleanup():
    print("\n--- Cleaning up test processes ---")
    for pid in reversed(pids):
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except Exception as e:
            print(f"Error killing {pid}: {e}")
    time.sleep(1)
    for pid in reversed(pids):
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

try:
    # 1. MicroXRCEAgent
    print("1. Starting MicroXRCEAgent...")
    cmd_agent = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888"
    proc_agent = subprocess.Popen(cmd_agent, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    pids.append(proc_agent.pid)
    time.sleep(2)

    # 2. Gazebo Server
    print("2. Starting Gazebo Harmonic headless on worlds/forest_sar.sdf...")
    env_gz = os.environ.copy()
    env_gz["GZ_IP"] = "127.0.0.1"
    env_gz["PX4_GZ_WORLD"] = "forest_sar"
    env_gz["PX4_GZ_WORLDS"] = f"{WORKSPACE_DIR}/worlds"
    env_gz["GZ_SIM_RESOURCE_PATH"] = f"{WORKSPACE_DIR}/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE_DIR}/assets_real/victims:{WORKSPACE_DIR}/generated_worlds:{WORKSPACE_DIR}/worlds"
    env_gz["GZ_SIM_SERVER_CONFIG_PATH"] = "/home/capstone/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config"

    cmd_gz = f"gz sim -s -v 3 -r {WORKSPACE_DIR}/worlds/forest_sar.sdf"
    proc_gz = subprocess.Popen(cmd_gz, shell=True, env=env_gz, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/gz_diag.log", "w"), stderr=subprocess.STDOUT)
    pids.append(proc_gz.pid)

    # Wait for clock
    print("   Waiting up to 15s for Gazebo clock...")
    clock_ready = False
    for _ in range(15):
        res = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], capture_output=True, text=True, timeout=3)
        if res.returncode == 0 and len(res.stdout.strip()) > 0:
            clock_ready = True
            break
        time.sleep(1)
    assert clock_ready, "Gazebo clock failed to start!"
    print("   Gazebo clock is active.")

    # 3. Start PX4 Instance 0
    print("3. Starting PX4 instance 0...")
    inst_dir = "/tmp/px4_diag_instance_0"
    os.makedirs(inst_dir, exist_ok=True)
    
    px4_env = env_gz.copy()
    px4_env["PX4_GZ_WORLD"] = "forest_sar"
    px4_env["PX4_GZ_MODEL"] = "x500_rgbd_down"
    px4_env["PX4_GZ_MODEL_POSE"] = "0,0,0.5,0,0,0"
    px4_env["PX4_UXRCE_DDS_NS"] = "drone_0"
    px4_env["PX4_GZ_STANDALONE"] = "1"
    px4_env["PX4_PARAM_NAV_DLL_ACT"] = "0"
    px4_env["PX4_SYS_AUTOSTART"] = "4100"
    px4_env["PX4_SIMULATOR"] = "gz"
    
    cmd_px4 = f"cd {inst_dir} && {BUILD_DIR}/bin/px4 -i 0 -d {BUILD_DIR}/etc"
    proc_px4 = subprocess.Popen(cmd_px4, shell=True, env=px4_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"{inst_dir}/out.log", "w"), stderr=open(f"{inst_dir}/err.log", "w"))
    pids.append(proc_px4.pid)

    print("   Waiting 15s for PX4 to initialize sensors and EKF2...")
    time.sleep(15)

    # 4. Check Gazebo Topics
    print("\n--- Gazebo Transport Sensor Topics Check ---")
    gz_topics_res = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=5)
    gz_topics = gz_topics_res.stdout.splitlines()
    for kw in ["imu", "air_pressure", "magnetometer", "navsat"]:
        matching = [t for t in gz_topics if kw in t]
        print(f"   [{kw.upper()}]: found {len(matching)} topics -> {matching[:2]}")

    # 5. Check ROS 2 Topics
    print("\n--- ROS 2 Topics Check ---")
    ros_cmd = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && ros2 topic list"
    ros_topics_res = subprocess.run(ros_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=5)
    ros_topics = ros_topics_res.stdout.splitlines()
    for target in ["/drone_0/fmu/out/vehicle_odometry", "/drone_0/fmu/out/sensor_combined", "/drone_0/fmu/out/vehicle_status"]:
        present = target in ros_topics
        print(f"   {target}: {'PRESENT' if present else 'MISSING'}")

    # Check vehicle_odometry samples
    if "/drone_0/fmu/out/vehicle_odometry" in ros_topics:
        echo_cmd = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && timeout 4 ros2 topic echo /drone_0/fmu/out/vehicle_odometry --once"
        echo_res = subprocess.run(echo_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
        print("\n--- Odometry Sample ---")
        for line in echo_res.stdout.splitlines()[:15]:
            print("   " + line)

    # 6. Check PX4 Log
    print("\n--- PX4 Log Preflight & Home Status ---")
    with open(f"{inst_dir}/out.log") as f:
        log_content = f.read()
    
    # Check for errors
    error_keywords = ["Accel Sensor 0 missing", "Gyro Sensor 0 missing", "barometer missing", "compass", "Preflight Fail", "CRITICAL"]
    found_errors = []
    for line in log_content.splitlines():
        for kw in error_keywords:
            if kw.lower() in line.lower() and "ready" not in line.lower():
                found_errors.append(line.strip())
                break
    
    if found_errors:
        print("   WARNING - Found sensor/preflight error lines in PX4 log:")
        for err in found_errors[:10]:
            print("     " + err)
    else:
        print("   EXCELLENT: No preflight sensor errors found in PX4 log!")

    # Check home position
    home_lines = [l.strip() for l in log_content.splitlines() if "home" in l.lower()]
    print(f"   Home position mentions: {home_lines}")

finally:
    cleanup()
