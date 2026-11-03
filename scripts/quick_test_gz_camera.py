#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import signal

WORKSPACE = "/home/capstone/capstone_project_antigravity"

gz_env = os.environ.copy()
gz_env["GZ_IP"] = "127.0.0.1"
gz_env["GZ_SIM_RESOURCE_PATH"] = (
    f"{WORKSPACE}/models:{WORKSPACE}/assets_real:{WORKSPACE}/assets_real/victims:"
    f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
    f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE}/generated_worlds:{WORKSPACE}/worlds"
)

test_sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.9">
<world name="test_camera_world">
  <physics type="ode">
    <max_step_size>0.004</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  <gravity>0 0 -9.8</gravity>
  <scene>
    <ambient>0.65 0.65 0.65 1</ambient>
    <background>0.72 0.84 0.97 1</background>
  </scene>
  <light name="sun" type="directional">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 200 0 0 0</pose>
    <diffuse>1 1 1 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.4 0.2 -1</direction>
  </light>
  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
      </visual>
    </link>
  </model>

  <include>
    <name>victim_0</name>
    <uri>model://casual_female</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>

  <model name="camera_drone">
    <static>true</static>
    <pose>0 0 6 0 1.5707 0</pose>
    <link name="camera_link">
      <sensor name="camera" type="rgbd_camera">
        <camera>
          <horizontal_fov>1.74</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.1</near><far>40.0</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>5</update_rate>
      </sensor>
    </link>
  </model>
</world>
</sdf>
"""

test_sdf_path = "/tmp/test_cam_world.sdf"
with open(test_sdf_path, "w") as f:
    f.write(test_sdf_content)

print("Starting Gazebo with test camera...")
cmd = f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -s -r --headless-rendering {test_sdf_path}"
proc = subprocess.Popen(cmd, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid)

try:
    time.sleep(5)
    print("Checking Gazebo topics...")
    res = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=5)
    print("Gazebo topics found:")
    for line in res.stdout.splitlines():
        if "camera" in line:
            print("  ->", line)
finally:
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        time.sleep(1)
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        pass
    print("Gazebo terminated.")
