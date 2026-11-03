#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import signal
import numpy as np

WORKSPACE = "/home/capstone/capstone_project_antigravity"

gz_env = os.environ.copy()
gz_env["GZ_IP"] = "127.0.0.1"
gz_env["GZ_SIM_RESOURCE_PATH"] = (
    f"{WORKSPACE}/models:{WORKSPACE}/assets_real:{WORKSPACE}/assets_real/victims:"
    f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
    f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE}/generated_worlds:{WORKSPACE}/worlds"
)

test_sdf_path = "/tmp/test_cam_world.sdf"

pids = []
def cleanup():
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

try:
    print("[1] Starting Gazebo...")
    cmd_gz = f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -s -r --headless-rendering {test_sdf_path}"
    p_gz = subprocess.Popen(cmd_gz, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid)
    pids.append(p_gz.pid)
    time.sleep(6)

    print("[2] Starting ros_gz_bridge for color, depth, and camera_info...")
    topics = (
        "/world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image "
        "/world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image "
        "/world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
    )
    remaps = (
        "--ros-args "
        "-r /world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/image:=/drone_0/camera/image "
        "-r /world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/depth_image:=/drone_0/camera/depth_image "
        "-r /world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/camera_info:=/drone_0/camera/camera_info"
    )
    cmd_br = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge {topics} {remaps}"
    p_br = subprocess.Popen(cmd_br, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    pids.append(p_br.pid)
    time.sleep(3)

    print("[3] Capturing color, depth, and camera_info via ROS 2...")
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge

    rclpy.init()
    bridge = CvBridge()
    frames = {"color": None, "depth": None, "info": None}

    sub_node = Node('test_stream_capture')
    sub_node.create_subscription(Image, '/drone_0/camera/image', lambda m: frames.update({"color": m}), 10)
    sub_node.create_subscription(Image, '/drone_0/camera/depth_image', lambda m: frames.update({"depth": m}), 10)
    sub_node.create_subscription(CameraInfo, '/drone_0/camera/camera_info', lambda m: frames.update({"info": m}), 10)

    t0 = time.time()
    while (frames["color"] is None or frames["depth"] is None or frames["info"] is None) and time.time() - t0 < 10.0:
        rclpy.spin_once(sub_node, timeout_sec=0.2)

    sub_node.destroy_node()
    rclpy.shutdown()

    if frames["color"] and frames["depth"] and frames["info"]:
        print(f"Color: {frames['color'].width}x{frames['color'].height}, enc: {frames['color'].encoding}")
        print(f"Depth: {frames['depth'].width}x{frames['depth'].height}, enc: {frames['depth'].encoding}")
        print(f"CameraInfo K: fx={frames['info'].k[0]:.2f}, fy={frames['info'].k[4]:.2f}, cx={frames['info'].k[2]:.2f}, cy={frames['info'].k[5]:.2f}")
        
        cv_depth = bridge.imgmsg_to_cv2(frames["depth"], desired_encoding="passthrough")
        valid = (cv_depth > 0) & (~np.isnan(cv_depth)) & (~np.isinf(cv_depth))
        print(f"Depth stats: min={np.min(cv_depth[valid]):.3f}m, median={np.median(cv_depth[valid]):.3f}m, max={np.max(cv_depth[valid]):.3f}m")
        # Sample at image center (where victim is)
        center_depth = cv_depth[240, 320]
        print(f"Center depth at (320, 240): {center_depth:.3f}m (Camera was at z=6m, victim height ~1.77m, ground=0m)")
    else:
        print("Failed to capture all streams! Captured:", {k: v is not None for k, v in frames.items()})

finally:
    cleanup()
    print("Cleaned up processes.")
