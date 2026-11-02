import os
import sys
import time
import subprocess
import signal

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
ARTIFACTS_DIR = "/home/capstone/.gemini/antigravity-ide/brain/bd5c4086-673f-46b0-a648-f6f98afc3ff9"

# Create a test world with a ground plane, directional light, camera, and the candidate tree models
test_world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="tree_test_world">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0.8 -0.8</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.2 0.35 0.18 1</ambient>
            <diffuse>0.25 0.42 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 1. Open Robotics Pine Tree -->
    <model name="test_pine_openrobotics">
      <static>true</static>
      <pose>-7 0 0 0 0 0</pose>
      <link name="link">
        <visual name="branch">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/pine_tree/meshes/pine_tree.dae</uri>
              <submesh><name>Branch</name></submesh>
              <scale>1.0 1.0 1.0</scale>
            </mesh>
          </geometry>
          <material>
            <double_sided>true</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>file://{WORKSPACE_DIR}/assets_real/Trees/pine_tree/materials/textures/branch_2_diffuse.png</albedo_map></metal></pbr>
          </material>
        </visual>
        <visual name="bark">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/pine_tree/meshes/pine_tree.dae</uri>
              <submesh><name>Bark</name></submesh>
              <scale>1.0 1.0 1.0</scale>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>file://{WORKSPACE_DIR}/assets_real/Trees/pine_tree/materials/textures/bark_diffuse.png</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- 2. Open Robotics Oak Tree -->
    <model name="test_oak_openrobotics">
      <static>true</static>
      <pose>-2 0 0 0 0 0</pose>
      <link name="link">
        <visual name="branch">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/oak_tree/meshes/oak_tree.dae</uri>
              <submesh><name>Branch</name></submesh>
              <scale>1.0 1.0 1.0</scale>
            </mesh>
          </geometry>
          <material>
            <double_sided>true</double_sided>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>file://{WORKSPACE_DIR}/assets_real/Trees/oak_tree/materials/textures/branch_diffuse.png</albedo_map></metal></pbr>
          </material>
        </visual>
        <visual name="bark">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/oak_tree/meshes/oak_tree.dae</uri>
              <submesh><name>Bark</name></submesh>
              <scale>1.0 1.0 1.0</scale>
            </mesh>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>file://{WORKSPACE_DIR}/assets_real/Trees/oak_tree/materials/textures/bark_diffuse.png</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- 3. Kenney Tall Pine GLB with 90 deg roll -->
    <model name="test_pine_tall_glb">
      <static>true</static>
      <pose>2 0 0 1.5708 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/pine_tall_a.glb</uri>
              <scale>6.0 6.0 6.0</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- 4. Kenney Detailed Tree GLB with 90 deg roll -->
    <model name="test_detailed_glb">
      <static>true</static>
      <pose>6 0 0 1.5708 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/detailed_tree.glb</uri>
              <scale>6.0 6.0 6.0</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- 5. Kenney Stump & Bush with 90 deg roll -->
    <model name="test_stump_glb">
      <static>true</static>
      <pose>0 -3 0 1.5708 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/stump_old.glb</uri>
              <scale>4.0 4.0 4.0</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="test_bush_glb">
      <static>true</static>
      <pose>-4 -3 0 1.5708 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://{WORKSPACE_DIR}/assets_real/Trees/plant_bush.glb</uri>
              <scale>5.0 5.0 5.0</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Camera positioned to view all models -->
    <model name="test_camera">
      <static>true</static>
      <pose>0 -14 4 0 0.1 1.5708</pose>
      <link name="link">
        <sensor name="camera_sensor" type="camera">
          <camera>
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>/test_camera/image</topic>
        </sensor>
      </link>
    </model>

  </world>
</sdf>
"""

test_world_path = "/tmp/tree_test_world.sdf"
with open(test_world_path, "w") as f:
    f.write(test_world_sdf)

pids = []
def cleanup():
    for pid in reversed(pids):
        try: os.killpg(pid, signal.SIGTERM)
        except: pass
    time.sleep(1)
    for pid in reversed(pids):
        try: os.killpg(pid, signal.SIGKILL)
        except: pass

try:
    print("Launching Gazebo Harmonic on test world...")
    env_gz = os.environ.copy()
    env_gz["GZ_IP"] = "127.0.0.1"
    env_gz["GZ_SIM_RESOURCE_PATH"] = f"{WORKSPACE_DIR}/models:{WORKSPACE_DIR}/assets_real:{WORKSPACE_DIR}/assets"
    env_gz["GZ_SIM_SERVER_CONFIG_PATH"] = "/home/capstone/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config"

    cmd_gz = f"gz sim -s -r --headless-rendering -v 3 {test_world_path}"
    proc_gz = subprocess.Popen(cmd_gz, shell=True, env=env_gz, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/gz_tree_test.log", "w"), stderr=subprocess.STDOUT)
    pids.append(proc_gz.pid)

    time.sleep(8)

    print("Bridging camera...")
    bridge_cmd = "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /test_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
    proc_bridge = subprocess.Popen(bridge_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    pids.append(proc_bridge.pid)

    time.sleep(3)

    # ROS 2 subscriber to capture 1 frame
    save_script = f"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Cap(Node):
    def __init__(self):
        super().__init__('cap_node')
        self.sub = self.create_subscription(Image, '/test_camera/image', self.cb, 10)
        self.br = CvBridge()
        self.saved = False

    def cb(self, msg):
        if not self.saved:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            out_file = '{ARTIFACTS_DIR}/tree_models_comparison.png'
            cv2.imwrite(out_file, img)
            print("Successfully saved: " + out_file)
            self.saved = True
            rclpy.shutdown()

rclpy.init()
node = Cap()
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.2)
    if node.saved:
        break
"""
    with open('/tmp/cap_frame.py', 'w') as f:
        f.write(save_script)

    subprocess.run("source /opt/ros/jazzy/setup.bash && python3 /tmp/cap_frame.py", shell=True, executable="/bin/bash", timeout=15)

finally:
    cleanup()
