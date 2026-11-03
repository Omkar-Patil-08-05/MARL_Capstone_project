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

    print("[2] Starting ros_gz_bridge...")
    bridge_topic = "/world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
    bridge_remap = "--ros-args -r /world/test_camera_world/model/camera_drone/link/camera_link/sensor/camera/image:=/drone_0/camera/image"
    cmd_br = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge '{bridge_topic}' {bridge_remap}"
    p_br = subprocess.Popen(cmd_br, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    pids.append(p_br.pid)
    time.sleep(3)

    print("[3] Capturing frame with ROS 2 subscriber...")
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    rclpy.init()
    bridge = CvBridge()
    received_img = []

    sub_node = Node('test_img_capture')
    sub_node.create_subscription(Image, '/drone_0/camera/image', lambda msg: received_img.append(msg), 10)

    t0 = time.time()
    while len(received_img) == 0 and time.time() - t0 < 10.0:
        rclpy.spin_once(sub_node, timeout_sec=0.2)

    sub_node.destroy_node()
    rclpy.shutdown()

    if received_img:
        print(f"Captured ROS 2 Image! Size: {received_img[0].width}x{received_img[0].height}, encoding: {received_img[0].encoding}")
        cv_img = bridge.imgmsg_to_cv2(received_img[0], 'bgr8')
        cv2.imwrite('/tmp/test_casual_female_render.png', cv_img)
        print("Saved to /tmp/test_casual_female_render.png")

        print("[4] Running YOLOv8n inference on rendered image...")
        yolo_site_packages = '/home/capstone/yolo_venv/lib/python3.12/site-packages'
        if os.path.exists(yolo_site_packages) and yolo_site_packages not in sys.path:
            sys.path.insert(0, yolo_site_packages)
        from ultralytics import YOLO
        model = YOLO(f"{WORKSPACE}/models/yolov8n.pt")
        results = model(cv_img)
        for r in results:
            for b in r.boxes:
                cls_id = int(b.cls[0])
                conf = float(b.conf[0])
                cls_name = model.names[cls_id]
                bbox = [int(x) for x in b.xyxy[0]]
                print(f"  -> YOLO Detection: class={cls_name} (id={cls_id}), conf={conf:.3f}, bbox={bbox}")
    else:
        print("Failed to receive image!")

finally:
    cleanup()
    print("Cleaned up processes.")
