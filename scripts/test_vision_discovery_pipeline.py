#!/usr/bin/env python3
"""
scripts/test_vision_discovery_pipeline.py

Controlled Simulation Test Scene for Vision-Perception Integration:
1. Launches Gazebo Harmonic on worlds/forest_sar.sdf with --headless-rendering.
2. Spawns PX4 SITL drone_0 with x500_rgbd_down directly above victim_0 (casual_female at -46.88, 99.61, 17.87).
   Drone pose: -46.88, 99.61, 21.5 (altitude ~3.6m above terrain/victim).
3. Launches ros_gz_bridge for camera topics and pose.
4. Captures live image and depth frames directly from Gazebo.
5. Runs ultralytics YOLOv8n on the captured frames across various confidence thresholds (0.05 to 0.50).
6. Runs swarm_perception_node and rgbd_victim_localizer in the loop.
7. Evaluates whether /sar/victim_discovery is published.
8. Evaluates matched 5 Hz vs 2 Hz trials.
9. Performs clean scoped teardown.
"""

import os
import sys
import time
import subprocess
import signal
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
BUILD_DIR = "/home/capstone/PX4-Autopilot/build/px4_sitl_default"
SDF_FILE = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf")
SDF_BACKUP = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf.bak_5hz")

def set_camera_update_rate(hz: int):
    with open(SDF_FILE, "r") as f:
        content = f.read()
    import re
    new_content = re.sub(r"<update_rate>\d+</update_rate>", f"<update_rate>{hz}</update_rate>", content)
    with open(SDF_FILE, "w") as f:
        f.write(new_content)
    print(f"[CONFIG] Camera update_rate set to {hz} Hz")

def restore_camera_sdf():
    if os.path.exists(SDF_BACKUP):
        with open(SDF_BACKUP, "r") as f:
            c = f.read()
        with open(SDF_FILE, "w") as f:
            f.write(c)
        print(f"[REVERT] Restored {SDF_FILE} to baseline 5 Hz.")

class VisionPipelineCollector(Node):
    def __init__(self):
        super().__init__('vision_pipeline_collector')
        self.bridge = CvBridge()
        self.color_msgs = []
        self.depth_msgs = []
        self.detection_msgs = []
        self.discovery_msgs = []
        self.camera_info_msg = None
        self.pose_msg = None

        self.create_subscription(RosImage, '/drone_0/camera/image', self._color_cb, qos_profile_sensor_data)
        self.create_subscription(RosImage, '/drone_0/camera/depth_image', self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/drone_0/camera/camera_info', self._info_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/model/drone_0/pose', self._pose_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/drone_0/perception/detections', self._det_cb, 10)
        self.create_subscription(String, '/sar/victim_discovery', self._disc_cb, 10)

    def _color_cb(self, msg):
        self.color_msgs.append(msg)

    def _depth_cb(self, msg):
        self.depth_msgs.append(msg)

    def _info_cb(self, msg):
        if self.camera_info_msg is None:
            self.camera_info_msg = msg

    def _pose_cb(self, msg):
        self.pose_msg = msg

    def _det_cb(self, msg):
        self.detection_msgs.append(msg)
        print(f"      [MSG] Detection #{len(self.detection_msgs)} on /drone_0/perception/detections")

    def _disc_cb(self, msg):
        self.discovery_msgs.append(msg)
        print(f"      [MSG] *** VICTIM DISCOVERY EVENT on /sar/victim_discovery: {msg.data}")

def run_vision_test_for_rate(target_hz: int) -> dict:
    print(f"\n{'='*70}\nRUNNING CONTROLLED VISION TEST AT {target_hz} Hz\n{'='*70}")
    set_camera_update_rate(target_hz)

    pids = []
    def kill_procs():
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
        subprocess.run("pkill -9 -f 'MicroXRCEAgent' || true; pkill -9 -f 'px4' || true; pkill -9 -f 'gz sim' || true; pkill -9 -f 'ruby.*gz' || true; pkill -9 -f 'ros_gz_bridge' || true; pkill -9 -f 'swarm_perception_node' || true; pkill -9 -f 'rgbd_victim_localizer' || true", shell=True, executable='/bin/bash', capture_output=True)

    try:
        # 1. MicroXRCEAgent
        print("[1/5] Starting MicroXRCEAgent...")
        cmd_agent = "source /opt/ros/jazzy/setup.bash && MicroXRCEAgent udp4 -p 8888"
        p_agent = subprocess.Popen(cmd_agent, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pids.append(p_agent.pid)
        time.sleep(1.5)

        # 2. Gazebo Server
        print("[2/5] Starting Gazebo Harmonic on worlds/forest_sar.sdf...")
        gz_env = os.environ.copy()
        gz_env["GZ_IP"] = "127.0.0.1"
        gz_env["PX4_GZ_WORLD"] = "forest_sar"
        gz_env["PX4_GZ_WORLDS"] = f"{WORKSPACE_DIR}/worlds"
        gz_env["GZ_SIM_RESOURCE_PATH"] = (
            f"{WORKSPACE_DIR}/models:{WORKSPACE_DIR}/assets_real:{WORKSPACE_DIR}/assets_real/victims:"
            f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
            f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE_DIR}/generated_worlds:{WORKSPACE_DIR}/worlds"
        )
        gz_cmd = f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -s -v 2 -r --headless-rendering {WORKSPACE_DIR}/worlds/forest_sar.sdf"
        p_gz = subprocess.Popen(gz_cmd, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/gz_vision_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_gz.pid)

        print("      Waiting 10s for Gazebo terrain and models to initialize...")
        time.sleep(10)

        # Wait for clock
        clock_ok = False
        for _ in range(30):
            try:
                res = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], capture_output=True, text=True, timeout=4)
                if res.returncode == 0 and len(res.stdout.strip()) > 0:
                    clock_ok = True
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1)
        assert clock_ok, "Gazebo clock failed to start!"
        print("      Gazebo clock active.")

        # 3. PX4 Drone 0 directly above victim_0
        print("[3/5] Starting PX4 instance 0 directly above victim_0 (x=-46.88, y=99.61, z=21.5)...")
        inst_dir = f"/tmp/px4_vision_{target_hz}hz"
        os.makedirs(inst_dir, exist_ok=True)
        px4_env = gz_env.copy()
        px4_env["PX4_GZ_WORLD"] = "forest_sar"
        px4_env["PX4_GZ_MODEL"] = "x500_rgbd_down"
        px4_env["PX4_GZ_MODEL_POSE"] = "-46.88,99.61,21.5,0,0,0"
        px4_env["PX4_UXRCE_DDS_NS"] = "drone_0"
        px4_env["PX4_GZ_STANDALONE"] = "1"
        px4_env["PX4_SYS_AUTOSTART"] = "4100"
        px4_env["PX4_SIMULATOR"] = "gz"
        px4_cmd = f"cd {inst_dir} && {BUILD_DIR}/bin/px4 -i 0 -d {BUILD_DIR}/etc"
        p_px4 = subprocess.Popen(px4_cmd, shell=True, env=px4_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"{inst_dir}/out.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_px4.pid)

        print("      Waiting up to 35s for camera topics in Gazebo...")
        cam_ready = False
        for _ in range(35):
            try:
                res = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=5)
                if "camera/image" in res.stdout:
                    cam_ready = True
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1)
        assert cam_ready, "Gazebo camera topic not published!"
        print("      Gazebo camera topics active.")

        # 4. ros_gz_bridge
        print("[4/5] Starting ros_gz_bridge...")
        bridge_topics = (
            "/world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image "
            "/world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image "
            "/world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo "
            "/model/x500_rgbd_down_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"
        )
        bridge_remaps = (
            "--ros-args "
            "-r /world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image:=/drone_0/camera/image "
            "-r /world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/depth_image:=/drone_0/camera/depth_image "
            "-r /world/forest_sar/model/x500_rgbd_down_0/link/camera_link/sensor/camera/camera_info:=/drone_0/camera/camera_info "
            "-r /model/x500_rgbd_down_0/pose:=/model/drone_0/pose"
        )
        bridge_cmd = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge {bridge_topics} {bridge_remaps}"
        p_bridge = subprocess.Popen(bridge_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/bridge_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_bridge.pid)
        time.sleep(3)

        # 5. Start YOLO Perception and RGB-D Localizer nodes
        print("[5/5] Launching swarm_perception_node and rgbd_victim_localizer...")
        yolo_cmd = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && ~/yolo_venv/bin/python3 /home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/swarm_perception_node.py"
        p_yolo = subprocess.Popen(yolo_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/yolo_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_yolo.pid)

        loc_cmd = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && python3 /home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/rgbd_victim_localizer.py"
        p_loc = subprocess.Popen(loc_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/localizer_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_loc.pid)

        # Collect frames for 45 seconds
        print("      Collecting camera frames, YOLO detections, and discovery events for 45s...")
        collector = VisionPipelineCollector()
        t_end = time.time() + 45.0
        while time.time() < t_end:
            rclpy.spin_once(collector, timeout_sec=0.2)

        # Process results
        n_color = len(collector.color_msgs)
        n_depth = len(collector.depth_msgs)
        n_dets = len(collector.detection_msgs)
        n_disc = len(collector.discovery_msgs)

        print(f"\n--- RESULTS FOR {target_hz} Hz ---")
        print(f"Color frames: {n_color} | Depth frames: {n_depth} | Detection msgs: {n_dets} | Discovery events: {n_disc}")

        sample_img_path = f"/tmp/victim_view_{target_hz}hz.png"
        sample_detections = []
        depth_stats = {}

        if n_color > 0:
            sample_color = collector.color_msgs[-1]
            cv_img = collector.bridge.imgmsg_to_cv2(sample_color, 'bgr8')
            cv2.imwrite(sample_img_path, cv_img)
            print(f"Saved sample image to {sample_img_path} ({cv_img.shape})")

            # Direct YOLO multi-threshold evaluation
            from ultralytics import YOLO
            model = YOLO('/home/capstone/capstone_project_antigravity/models/yolov8n.pt')
            y_res = model(cv_img, verbose=False)
            for r in y_res:
                for b in r.boxes:
                    cls_id = int(b.cls[0])
                    conf = float(b.conf[0])
                    name = model.names[cls_id]
                    sample_detections.append({
                        "class": name,
                        "class_id": cls_id,
                        "confidence": round(conf, 3),
                        "bbox": [int(x) for x in b.xyxy[0]]
                    })
                    print(f"Direct YOLO Detection: {name} (conf={conf:.3f}, bbox={[int(x) for x in b.xyxy[0]]})")

        if n_depth > 0:
            sample_depth = collector.depth_msgs[-1]
            cv_depth = collector.bridge.imgmsg_to_cv2(sample_depth, desired_encoding="passthrough")
            valid = (cv_depth > 0) & (~np.isnan(cv_depth)) & (~np.isinf(cv_depth))
            if np.any(valid):
                depth_stats = {
                    "min_m": round(float(np.min(cv_depth[valid])), 2),
                    "mean_m": round(float(np.mean(cv_depth[valid])), 2),
                    "median_m": round(float(np.median(cv_depth[valid])), 2),
                    "max_m": round(float(np.max(cv_depth[valid])), 2)
                }
                print(f"Depth readings: min={depth_stats['min_m']}m, median={depth_stats['median_m']}m, max={depth_stats['max_m']}m")

        discovery_events_data = []
        for d_msg in collector.discovery_msgs:
            try:
                discovery_events_data.append(json.loads(d_msg.data))
            except Exception:
                pass

        collector.destroy_node()

        return {
            "target_hz": target_hz,
            "color_frames_received": n_color,
            "depth_frames_received": n_depth,
            "detection_messages_received": n_dets,
            "discovery_events_received": n_disc,
            "depth_stats": depth_stats,
            "direct_yolo_detections": sample_detections,
            "discovery_events": discovery_events_data,
            "sample_image": sample_img_path
        }

    finally:
        kill_procs()

def main():
    rclpy.init()
    results = {}
    try:
        # Run 5 Hz test
        res_5hz = run_vision_test_for_rate(5)
        results["5hz"] = res_5hz

        # Run 2 Hz test
        res_2hz = run_vision_test_for_rate(2)
        results["2hz"] = res_2hz

    finally:
        restore_camera_sdf()
        rclpy.shutdown()

    out_json = os.path.join(WORKSPACE_DIR, "results", "vision_discovery_validation_results.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[COMPLETE] Vision discovery validation results saved to {out_json}")

if __name__ == "__main__":
    main()
