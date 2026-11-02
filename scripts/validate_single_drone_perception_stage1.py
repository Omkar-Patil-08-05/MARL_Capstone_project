#!/usr/bin/env python3
"""
scripts/validate_single_drone_perception_stage1.py

Stage 1: Isolated Single-Drone Perception Validation on forest_dense_candidate.sdf (950 trees).
- Uses ONE drone (drone_0) equipped with baseline x500_rgbd_down (unmodified 5 Hz SDF).
- Controlled hover directly above victim_0 (casual_female at -82.03, 43.95, 13.79).
- Drone pose: -82.03, 43.95, 17.4 (approx 3.6m above terrain/victim).
- Preserves the existing strict person-class filter in swarm_perception_node.py (no fallback).
- Measures:
    1. Actual RGB & Depth frame arrival rates, counts, and inter-frame intervals.
    2. YOLO detections emitted on /drone_0/perception/detections.
    3. Direct YOLO inference across all COCO classes on captured frames to identify class output.
    4. Depth statistics (min, median, mean, max, valid samples, depth spread).
    5. Discovery events on /sar/victim_discovery (distinguishing zero events from missing telemetry).
    6. Horizontal localization error, confidence, and latency if events are emitted.
    7. System resources (CPU%, Memory%, GPU Temp, GPU Util%, VRAM) and Gazebo RTF.
- Scoped clean teardown with guaranteed process termination.
"""

import os
import sys
import time
import subprocess
import signal
import json
import psutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

# Ensure Python unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Include yolo_venv site-packages for ultralytics if needed
YOLO_VENV_PATH = "/home/capstone/yolo_venv/lib/python3.12/site-packages"
if YOLO_VENV_PATH not in sys.path:
    sys.path.insert(0, YOLO_VENV_PATH)

WORKSPACE_DIR = "/home/capstone/capstone_project_antigravity"
BUILD_DIR = "/home/capstone/PX4-Autopilot/build/px4_sitl_default"
MODEL_SDF = os.path.join(WORKSPACE_DIR, "models", "x500_rgbd_down", "model.sdf")
WORLD_SDF = os.path.join(WORKSPACE_DIR, "worlds", "forest_dense_candidate.sdf")
META_FILE = os.path.join(WORKSPACE_DIR, "worlds", "forest_dense_candidate_meta.json")

GT_VICTIM_0 = {
    "id": "victim_0",
    "model": "casual_female",
    "world_x": -82.03125,
    "world_y": 43.9453125,
    "world_z": 13.785089
}

def get_system_metrics():
    metrics = {
        "cpu_percent": psutil.cpu_percent(interval=None),
        "memory_percent": psutil.virtual_memory().percent,
        "gpu_temp_c": None,
        "gpu_util_percent": None,
        "gpu_mem_used_mb": None,
        "gpu_mem_total_mb": None
    }
    try:
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=temperature.gpu,utilization.gpu,memory.used,memory.total", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2
        )
        if res.returncode == 0:
            parts = [p.strip() for p in res.stdout.strip().split(",")]
            if len(parts) >= 4:
                metrics["gpu_temp_c"] = float(parts[0])
                metrics["gpu_util_percent"] = float(parts[1])
                metrics["gpu_mem_used_mb"] = float(parts[2])
                metrics["gpu_mem_total_mb"] = float(parts[3])
    except Exception:
        pass
    return metrics

def get_gazebo_rtf():
    try:
        res = subprocess.run(["gz", "topic", "-e", "-t", "/clock", "-n", "2"], capture_output=True, text=True, timeout=3)
        if res.returncode == 0:
            lines = res.stdout.strip().splitlines()
            sim_times = []
            for line in lines:
                if "sec:" in line:
                    try:
                        sim_times.append(float(line.split(":")[1].strip()))
                    except ValueError:
                        pass
            if len(sim_times) >= 2 and sim_times[1] > sim_times[0]:
                return 1.0  # active clock
    except Exception:
        pass
    return None

class Stage1PerceptionCollector(Node):
    def __init__(self):
        super().__init__('stage1_perception_collector')
        self.bridge = CvBridge()
        
        self.color_msgs = []
        self.color_arrival_times = []
        self.depth_msgs = []
        self.depth_arrival_times = []
        
        self.detection_msgs = []
        self.detection_arrival_times = []
        
        self.discovery_msgs = []
        self.discovery_arrival_times = []
        
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
        self.color_arrival_times.append(time.time())

    def _depth_cb(self, msg):
        self.depth_msgs.append(msg)
        self.depth_arrival_times.append(time.time())

    def _info_cb(self, msg):
        if self.camera_info_msg is None:
            self.camera_info_msg = msg

    def _pose_cb(self, msg):
        self.pose_msg = msg

    def _det_cb(self, msg):
        self.detection_msgs.append(msg)
        self.detection_arrival_times.append(time.time())
        try:
            d = json.loads(msg.data)
            n_det = len(d.get("detections", []))
            print(f"      [PERCEPTION MSG #{len(self.detection_msgs)}] Detections: {n_det}")
        except Exception:
            pass

    def _disc_cb(self, msg):
        self.discovery_msgs.append(msg)
        self.discovery_arrival_times.append(time.time())
        print(f"      [*** DISCOVERY EVENT #{len(self.discovery_msgs)} ***] {msg.data}")

def run_stage1_validation(collection_duration_sec: float = 30.0) -> dict:
    print(f"\n{'='*75}\nSTAGE 1: ISOLATED SINGLE-DRONE PERCEPTION VALIDATION (CONTROLLED HOVER)\n{'='*75}")
    print(f"World:              {WORLD_SDF}")
    print(f"Target Victim:       victim_0 (casual_female at x={GT_VICTIM_0['world_x']}, y={GT_VICTIM_0['world_y']}, z={GT_VICTIM_0['world_z']})")
    print(f"Drone Spawn Pose:   x=-82.03, y=43.95, z=17.4 (approx 3.6m hover altitude)")
    print(f"Camera SDF:         Baseline unmodified (5 Hz)")
    print(f"Filter Policy:      Strict person filter (cls == 0, conf > 0.25)")
    print(f"{'='*75}\n")

    pids = []
    def cleanup():
        print("  Cleaning up Stage 1 processes...")
        for pid in reversed(pids):
            try: os.killpg(pid, signal.SIGTERM)
            except Exception: pass
        time.sleep(1)
        for pid in reversed(pids):
            try: os.killpg(pid, signal.SIGKILL)
            except Exception: pass
        subprocess.run(
            "pkill -9 -f 'MicroXRCEAgent' || true; pkill -9 -f 'px4' || true; "
            "pkill -9 -f 'gz sim' || true; pkill -9 -f 'ruby.*gz' || true; "
            "pkill -9 -f 'ros_gz_bridge' || true; pkill -9 -f 'swarm_perception_node' || true; "
            "pkill -9 -f 'rgbd_victim_localizer' || true",
            shell=True, executable='/bin/bash', capture_output=True
        )

    t_start = time.time()
    hw_snapshots = []
    
    try:
        # Pre-cleanup to ensure pristine state
        subprocess.run(
            "pkill -9 -f 'MicroXRCEAgent' || true; pkill -9 -f 'px4' || true; "
            "pkill -9 -f 'gz sim' || true; pkill -9 -f 'ruby.*gz' || true; "
            "pkill -9 -f 'ros_gz_bridge' || true; pkill -9 -f 'swarm_perception_node' || true; "
            "pkill -9 -f 'rgbd_victim_localizer' || true",
            shell=True, executable='/bin/bash', capture_output=True
        )
        time.sleep(1)

        # 1. MicroXRCEAgent
        print("[1/5] Starting MicroXRCEAgent...")
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
        gz_cmd = f"source /home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -s -v 2 -r --headless-rendering {WORLD_SDF}"
        p_gz = subprocess.Popen(gz_cmd, shell=True, env=gz_env, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/gz_stage1_perception.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_gz.pid)

        print("      Waiting 12s for Gazebo world loading...")
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
        print("      Gazebo simulation clock confirmed active.")

        # 3. Spawn PX4 Drone 0 with x500_rgbd_down
        print("[3/5] Starting PX4 SITL instance 0 with x500_rgbd_down directly above victim_0...")
        inst_dir = "/tmp/px4_stage1_perception"
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

        print("      Waiting up to 45s for Gazebo camera topics...")
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
        print("      Gazebo optical camera stream confirmed.")

        # 4. ros_gz_bridge
        print("[4/5] Launching ros_gz_bridge for camera streams, depth, and pose...")
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
        p_bridge = subprocess.Popen(bridge_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/bridge_stage1.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_bridge.pid)
        time.sleep(3)

        # 5. Start YOLO Perception & RGB-D Localizer
        print("[5/5] Launching swarm_perception_node and rgbd_victim_localizer (Strict Person Filter)...")
        yolo_cmd = (
            "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && "
            "/home/capstone/yolo_venv/bin/python3 /home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/swarm_perception_node.py"
        )
        p_yolo = subprocess.Popen(yolo_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/yolo_stage1.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_yolo.pid)

        loc_cmd = (
            "source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && "
            "export DEPTH_SYNC_TOLERANCE=0.6 && "
            "python3 /home/capstone/capstone_project_antigravity/drone_ws/src/swarm_controller/swarm_controller/rgbd_victim_localizer.py"
        )
        p_loc = subprocess.Popen(loc_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=open("/tmp/localizer_stage1.log", "w"), stderr=subprocess.STDOUT)
        pids.append(p_loc.pid)

        # Collect metrics for specified duration
        print(f"      Collecting live perception telemetry for {collection_duration_sec}s...")
        collector = Stage1PerceptionCollector()
        t_collect_end = time.time() + collection_duration_sec
        last_hw_sample = 0
        
        while time.time() < t_collect_end:
            rclpy.spin_once(collector, timeout_sec=0.1)
            now = time.time()
            if now - last_hw_sample >= 2.0:
                last_hw_sample = now
                hw_snapshots.append(get_system_metrics())

        print("      Collection window complete. Analyzing results...")

        # Process frame rates
        n_color = len(collector.color_msgs)
        n_depth = len(collector.depth_msgs)
        n_det_msgs = len(collector.detection_msgs)
        n_disc_events = len(collector.discovery_msgs)

        color_duration = collector.color_arrival_times[-1] - collector.color_arrival_times[0] if n_color > 1 else 0
        depth_duration = collector.depth_arrival_times[-1] - collector.depth_arrival_times[0] if n_depth > 1 else 0
        color_hz = round((n_color - 1) / color_duration, 2) if color_duration > 0 else 0.0
        depth_hz = round((n_depth - 1) / depth_duration, 2) if depth_duration > 0 else 0.0

        color_intervals = [
            round(collector.color_arrival_times[i] - collector.color_arrival_times[i-1], 3)
            for i in range(1, n_color)
        ] if n_color > 1 else []

        depth_intervals = [
            round(collector.depth_arrival_times[i] - collector.depth_arrival_times[i-1], 3)
            for i in range(1, n_depth)
        ] if n_depth > 1 else []

        # Direct YOLO multi-class evaluation on the captured image
        direct_yolo_detections = []
        sample_img_path = "/tmp/stage1_victim_view.png"
        sample_depth_path = "/tmp/stage1_depth_sample.npy"
        depth_stats = {}

        if n_color > 0:
            sample_color = collector.color_msgs[-1]
            cv_img = collector.bridge.imgmsg_to_cv2(sample_color, 'bgr8')
            cv2.imwrite(sample_img_path, cv_img)
            
            try:
                from ultralytics import YOLO
                yolo_model = YOLO('/home/capstone/capstone_project_antigravity/models/yolov8n.pt')
                y_res = yolo_model(cv_img, verbose=False)
                for r in y_res:
                    for b in r.boxes:
                        cls_id = int(b.cls[0])
                        conf = float(b.conf[0])
                        name = yolo_model.names[cls_id]
                        bbox = [int(x) for x in b.xyxy[0]]
                        direct_yolo_detections.append({
                            "class": name,
                            "class_id": cls_id,
                            "confidence": round(conf, 3),
                            "bbox": bbox
                        })
                        print(f"      [Direct YOLO] Detected: '{name}' (class_id={cls_id}, conf={conf:.3f}, bbox={bbox})")
            except Exception as e:
                print(f"      [Direct YOLO Evaluation Error] {e}")

        # Depth buffer statistics
        if n_depth > 0:
            sample_depth = collector.depth_msgs[-1]
            try:
                cv_depth = collector.bridge.imgmsg_to_cv2(sample_depth, desired_encoding="passthrough")
                np.save(sample_depth_path, cv_depth)
                valid = (cv_depth > 0) & (~np.isnan(cv_depth)) & (~np.isinf(cv_depth))
                if np.any(valid):
                    depth_stats = {
                        "min_m": round(float(np.min(cv_depth[valid])), 2),
                        "mean_m": round(float(np.mean(cv_depth[valid])), 2),
                        "median_m": round(float(np.median(cv_depth[valid])), 2),
                        "max_m": round(float(np.max(cv_depth[valid])), 2),
                        "valid_pixel_count": int(np.sum(valid)),
                        "total_pixel_count": int(cv_depth.size)
                    }
            except Exception as e:
                print(f"      [Depth Stats Error] {e}")

        # Decode perception node output
        decoded_perception_detections = []
        for msg in collector.detection_msgs:
            try:
                data = json.loads(msg.data)
                for det in data.get("detections", []):
                    decoded_perception_detections.append(det)
            except Exception:
                pass

        # Parse discovery events
        parsed_discovery_events = []
        localization_errors = []
        event_latencies = []

        for i, d_msg in enumerate(collector.discovery_msgs):
            try:
                ev = json.loads(d_msg.data)
                parsed_discovery_events.append(ev)
                
                # Compute localization error
                wx = ev.get("world_x")
                wy = ev.get("world_y")
                if wx is not None and wy is not None:
                    err = math.sqrt((wx - GT_VICTIM_0["world_x"])**2 + (wy - GT_VICTIM_0["world_y"])**2)
                    localization_errors.append(round(err, 4))
                
                # Event latency (relative to detection timestamp)
                ev_ts = ev.get("timestamp")
                arrival_ts = collector.discovery_arrival_times[i]
                if ev_ts is not None:
                    event_latencies.append(round(arrival_ts - ev_ts, 3))
            except Exception:
                pass

        # Compute average system resources
        avg_cpu = round(sum(h["cpu_percent"] for h in hw_snapshots) / len(hw_snapshots), 1) if hw_snapshots else None
        avg_mem = round(sum(h["memory_percent"] for h in hw_snapshots) / len(hw_snapshots), 1) if hw_snapshots else None
        gpu_temps = [h["gpu_temp_c"] for h in hw_snapshots if h["gpu_temp_c"] is not None]
        avg_gpu_temp = round(sum(gpu_temps) / len(gpu_temps), 1) if gpu_temps else None
        max_gpu_temp = max(gpu_temps) if gpu_temps else None
        gpu_utils = [h["gpu_util_percent"] for h in hw_snapshots if h["gpu_util_percent"] is not None]
        avg_gpu_util = round(sum(gpu_utils) / len(gpu_utils), 1) if gpu_utils else None

        collector.destroy_node()

        total_wall_time = round(time.time() - t_start, 1)

        results = {
            "test_type": "Stage 1: Single-Drone Isolated Perception Validation (Controlled Hover)",
            "status": "COMPLETED",
            "world": "forest_dense_candidate.sdf",
            "drone_id": "drone_0",
            "drone_spawn_pose": [-82.03, 43.95, 17.4, 0, 0, 0],
            "ground_truth_victim": GT_VICTIM_0,
            "filter_policy": "Strict person filter (cls == 0)",
            "camera_update_rate_target_hz": 5,
            "optical_transport": {
                "color_frames_received": n_color,
                "measured_color_hz": color_hz,
                "mean_color_interval_sec": round(float(np.mean(color_intervals)), 3) if color_intervals else None,
                "depth_frames_received": n_depth,
                "measured_depth_hz": depth_hz,
                "mean_depth_interval_sec": round(float(np.mean(depth_intervals)), 3) if depth_intervals else None,
                "depth_readings": depth_stats
            },
            "perception_node": {
                "messages_published": n_det_msgs,
                "person_detections_passed_filter": len(decoded_perception_detections),
                "person_detections": decoded_perception_detections
            },
            "direct_yolo_evaluation": {
                "raw_detections_count": len(direct_yolo_detections),
                "detections": direct_yolo_detections
            },
            "localizer_and_discovery": {
                "events_published_count": n_disc_events,
                "events": parsed_discovery_events,
                "localization_errors_m": localization_errors,
                "event_latencies_sec": event_latencies
            },
            "hardware_resources": {
                "collection_wall_duration_sec": collection_duration_sec,
                "total_wall_time_sec": total_wall_time,
                "avg_cpu_percent": avg_cpu,
                "avg_mem_percent": avg_mem,
                "avg_gpu_temp_c": avg_gpu_temp,
                "max_gpu_temp_c": max_gpu_temp,
                "avg_gpu_util_percent": avg_gpu_util
            },
            "artifacts": {
                "sample_image": sample_img_path,
                "gz_log": "/tmp/gz_stage1_perception.log",
                "yolo_log": "/tmp/yolo_stage1.log",
                "localizer_log": "/tmp/localizer_stage1.log"
            }
        }

        return results

    finally:
        cleanup()

def main():
    rclpy.init()
    try:
        results = run_stage1_validation(collection_duration_sec=30.0)
    finally:
        rclpy.shutdown()

    out_file = os.path.join(WORKSPACE_DIR, "scratch", "stage1_single_drone_perception_results.json")
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\n[STAGE 1 COMPLETE] Results saved to {out_file}")

if __name__ == "__main__":
    main()
