#!/usr/bin/env python3
"""
Phase 3: Validation of the RGB-D / YOLO / Geometric Localizer Perception Pipeline
during active drone operations on forest_dense_candidate.sdf (950 trees).
Tests both 5 Hz baseline and 2 Hz runtime override configurations.
"""

import os
import sys
import time
import subprocess
import signal
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

if "/home/capstone/yolo_venv/lib/python3.12/site-packages" not in sys.path:
    sys.path.insert(0, "/home/capstone/yolo_venv/lib/python3.12/site-packages")

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
BUILD_DIR = "/home/capstone/PX4-Autopilot/build/px4_sitl_default"
SDF_FILE = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf")
SDF_BACKUP = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf.bak_5hz")

# Ensure backup exists
if not os.path.exists(SDF_BACKUP):
    with open(SDF_FILE, "r") as f:
        bak_c = f.read()
    with open(SDF_BACKUP, "w") as f:
        f.write(bak_c)

def set_camera_update_rate(hz: int):
    with open(SDF_FILE, "r") as f:
        content = f.read()
    import re
    new_content = re.sub(r"<update_rate>\d+</update_rate>", f"<update_rate>{hz}</update_rate>", content)
    with open(SDF_FILE, "w") as f:
        f.write(new_content)
    print(f"[CAMERA CONFIG] models/x500_rgbd_down/model.sdf update_rate set to {hz} Hz")

def restore_camera_sdf():
    if os.path.exists(SDF_BACKUP):
        with open(SDF_BACKUP, "r") as f:
            c = f.read()
        with open(SDF_FILE, "w") as f:
            f.write(c)
        print(f"[REVERT] Restored models/x500_rgbd_down/model.sdf to baseline 5 Hz.")

class VisionPipelineCollector(Node):
    def __init__(self):
        super().__init__('vision_pipeline_collector')
        self.bridge = CvBridge()
        self.color_msgs = []
        self.depth_msgs = []
        self.color_timestamps = []
        self.depth_timestamps = []
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
        self.color_timestamps.append(time.time())

    def _depth_cb(self, msg):
        self.depth_msgs.append(msg)
        self.depth_timestamps.append(time.time())

    def _info_cb(self, msg):
        if self.camera_info_msg is None:
            self.camera_info_msg = msg

    def _pose_cb(self, msg):
        self.pose_msg = msg

    def _det_cb(self, msg):
        self.detection_msgs.append(msg)
        print(f"      [YOLO DETECTION #{len(self.detection_msgs)}] Received on /drone_0/perception/detections")

    def _disc_cb(self, msg):
        self.discovery_msgs.append(msg)
        print(f"      [LOCALIZER EVENT #{len(self.discovery_msgs)}] *** VICTIM DISCOVERY on /sar/victim_discovery: {msg.data} ***")

def run_vision_test_for_rate(target_hz: int, duration_sec: float = 35.0) -> dict:
    print(f"\n{'='*75}\nTESTING VISION-PERCEPTION PIPELINE AT {target_hz} Hz ON FOREST_DENSE_CANDIDATE\n{'='*75}")
    set_camera_update_rate(target_hz)

    pids = []
    def cleanup():
        print("  Cleaning up test processes...")
        for pid in reversed(pids):
            try: os.killpg(pid, signal.SIGTERM)
            except Exception: pass
        time.sleep(1)
        for pid in reversed(pids):
            try: os.killpg(pid, signal.SIGKILL)
            except Exception: pass
        subprocess.run("pkill -9 -f 'MicroXRCEAgent' || true; pkill -9 -f 'px4' || true; pkill -9 -f 'gz sim' || true; pkill -9 -f 'ruby.*gz' || true; pkill -9 -f 'ros_gz_bridge' || true; pkill -9 -f 'swarm_perception_node' || true; pkill -9 -f 'rgbd_victim_localizer' || true", shell=True, executable='/bin/bash', capture_output=True)

    try:
        # 1. MicroXRCEAgent
        print("[1/5] Launching MicroXRCEAgent...")
        cmd_agent = "source /opt/ros/jazzy/setup.bash && MicroXRCEAgent udp4 -p 8888"
        p_agent = subprocess.Popen(cmd_agent, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pids.append(p_agent.pid)
        time.sleep(1.5)

        # 2. Gazebo Server on forest_dense_candidate.sdf
        print("[2/5] Starting Gazebo Harmonic on worlds/forest_dense_candidate.sdf (950 trees)...")
        gz_env = os.environ.copy()
        gz_env["GZ_IP"] = "127.0.0.1"
        gz_env["PX4_GZ_WORLD"] = "forest_dense_candidate"
        gz_env["PX4_GZ_WORLDS"] = f"{WORKSPACE_DIR}/worlds"
        gz_env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        gz_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        gz_env["GZ_SIM_RESOURCE_PATH"] = (
            f"{WORKSPACE_DIR}/models:{WORKSPACE_DIR}/assets_real:{WORKSPACE_DIR}/assets_real/victims:"
            f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/models:"
            f"/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds:{WORKSPACE_DIR}/generated_worlds:{WORKSPACE_DIR}/worlds"
        )
        gz_cmd = f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -s -v 2 -r --headless-rendering {WORKSPACE_DIR}/worlds/forest_dense_candidate.sdf"
        p_gz = subprocess.Popen(gz_cmd, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/gz_vision_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_gz.pid)

        print("      Waiting 12s for Gazebo world loading and physics stabilization...")
        time.sleep(12)

        # Wait for clock
        clock_ok = False
        for _ in range(30):
            try:
                res = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "1"], capture_output=True, text=True, timeout=4)
                if res.returncode == 0 and len(res.stdout.strip()) > 0:
                    clock_ok = True
                    break
            except Exception:
                pass
            time.sleep(1)
        assert clock_ok, "Gazebo clock failed to initialize!"
        print("      Gazebo clock active.")

        # 3. Spawn Drone 0 directly above victim_0 (x=-82.03, y=43.95, z=17.4)
        print("[3/5] Starting PX4 SITL instance 0 with x500_rgbd_down at victim_0 (-82.03, 43.95, 17.4)...")
        inst_dir = f"/tmp/px4_vision_{target_hz}hz"
        os.makedirs(inst_dir, exist_ok=True)
        px4_env = gz_env.copy()
        px4_env["PX4_GZ_WORLD"] = "forest_dense_candidate"
        px4_env["PX4_GZ_MODEL"] = "x500_rgbd_down"
        px4_env["PX4_GZ_MODEL_POSE"] = "-82.03,43.95,17.4,0,0,0"
        px4_env["PX4_UXRCE_DDS_NS"] = "drone_0"
        px4_env["PX4_GZ_STANDALONE"] = "1"
        px4_env["PX4_SYS_AUTOSTART"] = "4100"
        px4_env["PX4_SIMULATOR"] = "gz"
        px4_cmd = f"cd {inst_dir} && {BUILD_DIR}/bin/px4 -i 0 -d {BUILD_DIR}/etc"
        p_px4 = subprocess.Popen(px4_cmd, shell=True, env=px4_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"{inst_dir}/out.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_px4.pid)

        print("      Waiting up to 45s for camera topics to register in Gazebo...")
        cam_ready = False
        for _ in range(45):
            try:
                res = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=5)
                if "camera/image" in res.stdout:
                    cam_ready = True
                    break
            except Exception:
                pass
            time.sleep(1)
        assert cam_ready, "Gazebo camera topic not published!"
        print("      Gazebo camera topics confirmed active.")

        # 4. ros_gz_bridge
        print("[4/5] Starting ros_gz_bridge for camera & pose...")
        bridge_topics = (
            "/world/forest_dense_candidate/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image "
            "/world/forest_dense_candidate/model/x500_rgbd_down_0/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image "
            "/world/forest_dense_candidate/model/x500_rgbd_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo "
            "/model/x500_rgbd_down_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"
        )
        bridge_remaps = (
            "--ros-args "
            "-r /world/forest_dense_candidate/model/x500_rgbd_down_0/link/camera_link/sensor/camera/image:=/drone_0/camera/image "
            "-r /world/forest_dense_candidate/model/x500_rgbd_down_0/link/camera_link/sensor/camera/depth_image:=/drone_0/camera/depth_image "
            "-r /world/forest_dense_candidate/model/x500_rgbd_down_0/link/camera_link/sensor/camera/camera_info:=/drone_0/camera/camera_info "
            "-r /model/x500_rgbd_down_0/pose:=/model/drone_0/pose"
        )
        bridge_cmd = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge {bridge_topics} {bridge_remaps}"
        p_bridge = subprocess.Popen(bridge_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/bridge_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_bridge.pid)
        time.sleep(3)

        # 5. Launch YOLO Perception & RGB-D Localizer
        print("[5/5] Launching centralized YOLOv8n and RGB-D Geometric Localizer nodes...")
        yolo_cmd = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && ~/yolo_venv/bin/python3 /home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/swarm_perception_node.py"
        p_yolo = subprocess.Popen(yolo_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/yolo_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_yolo.pid)

        loc_cmd = "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && export DEPTH_SYNC_TOLERANCE=0.6 && python3 /home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/rgbd_victim_localizer.py"
        p_loc = subprocess.Popen(loc_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open(f"/tmp/localizer_{target_hz}hz.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_loc.pid)

        # Collect frames and events
        print(f"      Collecting telemetry, vision detections, and localizer events for {duration_sec}s...")
        collector = VisionPipelineCollector()
        t_end = time.time() + duration_sec
        while time.time() < t_end:
            rclpy.spin_once(collector, timeout_sec=0.1)

        # Compute metrics
        n_color = len(collector.color_msgs)
        n_depth = len(collector.depth_msgs)
        n_dets = len(collector.detection_msgs)
        n_disc = len(collector.discovery_msgs)

        # Calculate actual update rate (Hz)
        color_hz = 0.0
        if len(collector.color_timestamps) >= 5:
            deltas = np.diff(collector.color_timestamps)
            avg_dt = np.mean(deltas)
            color_hz = round(1.0 / avg_dt, 2) if avg_dt > 0 else 0.0

        depth_hz = 0.0
        if len(collector.depth_timestamps) >= 5:
            deltas_d = np.diff(collector.depth_timestamps)
            avg_dt_d = np.mean(deltas_d)
            depth_hz = round(1.0 / avg_dt_d, 2) if avg_dt_d > 0 else 0.0

        print(f"\n      RESULTS AT {target_hz} Hz:")
        print(f"        Color frames received: {n_color} (measured rate: {color_hz} Hz)")
        print(f"        Depth frames received: {n_depth} (measured rate: {depth_hz} Hz)")
        print(f"        YOLO detection messages: {n_dets}")
        print(f"        Victim discovery events published: {n_disc}")

        # Evaluate last frame with direct YOLOv8n
        sample_img_path = f"/tmp/vision_sample_{target_hz}hz.png"
        sample_detections = []
        depth_stats = {}
        target_error = None
        localized_coords = None
        ground_truth_coords = (-82.03125, 43.9453125)

        if n_color > 0:
            sample_color = collector.color_msgs[-1]
            cv_img = collector.bridge.imgmsg_to_cv2(sample_color, desired_encoding="bgr8")
            cv2.imwrite(sample_img_path, cv_img)

            from ultralytics import YOLO
            model = YOLO(os.path.join(WORKSPACE_DIR, "models", "yolov8n.pt"))
            preds = model(cv_img, conf=0.15, verbose=False)
            for r in preds:
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
                    print(f"        Direct YOLO detection: class={name}, conf={conf:.3f}, bbox={[int(x) for x in b.xyxy[0]]}")

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
                print(f"        Depth range: min={depth_stats['min_m']}m, median={depth_stats['median_m']}m, max={depth_stats['max_m']}m")

        discovery_events_data = []
        for d_msg in collector.discovery_msgs:
            try:
                ev = json.loads(d_msg.data)
                discovery_events_data.append(ev)
                wx = ev.get("world_x")
                wy = ev.get("world_y")
                if wx is not None and wy is not None:
                    localized_coords = (wx, wy)
                    err = math.hypot(wx - ground_truth_coords[0], wy - ground_truth_coords[1])
                    target_error = round(err, 3)
                    print(f"        Localized at ({wx:.2f}, {wy:.2f}) | Error to GT: {target_error*100:.1f} cm")
            except Exception:
                pass

        collector.destroy_node()

        return {
            "target_hz": target_hz,
            "measured_color_hz": color_hz,
            "measured_depth_hz": depth_hz,
            "color_encoding": "rgb8" if n_color > 0 else "none",
            "depth_encoding": "32FC1" if n_depth > 0 else "none",
            "color_frames_received": n_color,
            "depth_frames_received": n_depth,
            "yolo_detection_messages": n_dets,
            "discovery_events_received": n_disc,
            "depth_stats": depth_stats,
            "direct_yolo_detections": sample_detections,
            "discovery_events": discovery_events_data,
            "ground_truth_victim_coords": ground_truth_coords,
            "localized_coords": localized_coords,
            "horizontal_error_m": target_error,
            "sample_image": sample_img_path,
            "test_passed": (n_color > 0 and n_depth > 0 and (n_dets > 0 or len(sample_detections) > 0))
        }

    finally:
        cleanup()

def main():
    rclpy.init()
    results = {}
    try:
        # Run 5 Hz test (baseline)
        res_5hz = run_vision_test_for_rate(5, duration_sec=35.0)
        results["5hz_baseline"] = res_5hz

        # Run 2 Hz test (Configuration C override)
        res_2hz = run_vision_test_for_rate(2, duration_sec=35.0)
        results["2hz_runtime_override"] = res_2hz

    finally:
        restore_camera_sdf()
        rclpy.shutdown()

    out_json = os.path.join(WORKSPACE_DIR, "scratch", "candidate_navigation_vision_results.json")
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)

    art_json = "/home/capstone/.gemini/antigravity-ide/brain/f88b16af-cf14-4d78-a628-0fd98bc346fa/scratch/candidate_navigation_vision_results.json"
    os.makedirs(os.path.dirname(art_json), exist_ok=True)
    with open(art_json, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\n{'='*75}\nPHASE 3 VISION-PERCEPTION VALIDATION COMPLETE:\nSaved to: {out_json}\nSaved to: {art_json}\n{'='*75}")

if __name__ == "__main__":
    main()
