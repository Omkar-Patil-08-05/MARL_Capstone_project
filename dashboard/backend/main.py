import asyncio
import json
import threading
import subprocess
import os
import signal
import time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from world import get_world_data, get_map_registry

app = FastAPI(title="Antigravity Live Telemetry")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# In-memory global state
_LATEST_TELEMETRY = None
_TELEMETRY_RECEIVED = False
_LAST_TIMESTAMP = None

# ---------------------------------------------------------
# WebSocket Manager
# ---------------------------------------------------------
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        if _LATEST_TELEMETRY:
            await websocket.send_text(_LATEST_TELEMETRY)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        failed_connections = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                failed_connections.append(connection)

        for fc in failed_connections:
            self.disconnect(fc)

manager = ConnectionManager()

# ---------------------------------------------------------
# Mission Manager
# ---------------------------------------------------------
class MissionManager:
    def __init__(self):
        self.state = "IDLE"
        self.active_map_id = None
        self.process_groups = []
        self.error_msg = None
        self.lock = threading.Lock()

    def set_state(self, new_state, error=None):
        with self.lock:
            self.state = new_state
            if error:
                self.error_msg = error
            print(f"[MISSION] Transition -> {new_state}")

    async def stop_mission(self):
        with self.lock:
            if self.state == "STOPPING":
                return
            self.state = "STOPPING"

        print("[MISSION] Stopping all process groups...")
        for pgid in self.process_groups:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception as e:
                print(f"Error killing pgid {pgid}: {e}")
        self.process_groups = []

        # Fallback system cleanup just to be completely safe
        subprocess.run('pkill -9 -f "MicroXRCEAgent" || true', shell=True)
        subprocess.run('pkill -9 -f "px4" || true', shell=True)
        subprocess.run('pkill -9 -f "gz sim" || true', shell=True)
        subprocess.run('pkill -9 -f "ruby.*gz" || true', shell=True)
        subprocess.run('pkill -9 -f "qmix_drone_test" || true', shell=True)
        subprocess.run('pkill -9 -f "swarm_runner" || true', shell=True)
        subprocess.run('pkill -9 -f "yolo_human_detection" || true', shell=True)

        # Stop ros2 daemon to clear phantom topic caches
        subprocess.run('source /opt/ros/jazzy/setup.bash && ros2 daemon stop || true', shell=True, executable='/bin/bash')

        # Wait for processes to genuinely terminate
        for _ in range(15):
            res = subprocess.run('pgrep -f "gz sim|px4|MicroXRCEAgent|swarm_runner|yolo_human_detection"', shell=True, capture_output=True, text=True)
            if not res.stdout.strip():
                break
            await asyncio.sleep(1)

        global _LATEST_TELEMETRY, _TELEMETRY_RECEIVED
        _LATEST_TELEMETRY = None
        _TELEMETRY_RECEIVED = False

        with self.lock:
            self.state = "IDLE"
            self.active_map_id = None

mission_manager = MissionManager()

async def start_mission_task(map_id: str, drone_count: int):
    try:
        mission_manager.set_state("STARTING")
        mission_manager.active_map_id = map_id

        # 1. Start Simulator Stack
        launch_script = os.path.expanduser("~/capstone_project_antigravity/scripts/launch_swarm.sh")
        # Start in new process group
        sim_proc = subprocess.Popen(["bash", launch_script, map_id, str(drone_count)], preexec_fn=os.setsid)
        mission_manager.process_groups.append(os.getpgid(sim_proc.pid))

        # 2. Wait for Simulator Topics (up to 90s)
        print("[MISSION] Waiting for /drone_0/fmu/out/vehicle_odometry...")
        ready = False
        for _ in range(30):
            if mission_manager.state != "STARTING":
                return
            res = subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 topic list"], capture_output=True, text=True)
            if all(f"/drone_{i}/fmu/out/vehicle_odometry" in res.stdout for i in range(drone_count)):
                ready = True
                break
            await asyncio.sleep(3)

        if not ready:
            mission_manager.set_state("ERROR", "Timed out waiting for simulator topics.")
            await mission_manager.stop_mission()
            return

        mission_manager.set_state("SIMULATOR_READY")

        # 3. Wait for EKF2 stabilization
        print("[MISSION] Waiting 60s for EKF2 stabilization...")
        for _ in range(60):
            if mission_manager.state != "SIMULATOR_READY":
                return
            await asyncio.sleep(1)

        mission_manager.set_state("QMIX_STARTING")

        # 4. Start QMIX Controller
        qmix_cmd = f"source /opt/ros/jazzy/setup.bash && source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash && export ROS_LOCALHOST_ONLY=1 && ros2 run swarm_controller swarm_runner --map {map_id} --drones {drone_count}"
        qmix_proc = subprocess.Popen(["bash", "-c", qmix_cmd], preexec_fn=os.setsid)
        mission_manager.process_groups.append(os.getpgid(qmix_proc.pid))

        mission_manager.set_state("RUNNING")

    except Exception as e:
        mission_manager.set_state("ERROR", str(e))
        await mission_manager.stop_mission()


# ---------------------------------------------------------
# ROS 2 Node (Runs in background thread)
# ---------------------------------------------------------
class TelemetrySubscriber(Node):
    def __init__(self, loop):
        super().__init__('telemetry_subscriber_backend')
        self.loop = loop
        self.subscription = self.create_subscription(String, '/swarm/telemetry', self.listener_callback, 10)

    def listener_callback(self, msg):
        global _LATEST_TELEMETRY, _TELEMETRY_RECEIVED, _LAST_TIMESTAMP
        try:
            data = json.loads(msg.data)
            if data.get("type") == "telemetry":
                # Enforce monotonic telemetry (ignore stale publishers)
                msg_timestamp = data.get("timestamp", 0)
                if _LAST_TIMESTAMP is not None and msg_timestamp < _LAST_TIMESTAMP:
                    return

                # Inject active map into telemetry
                data["active_map_id"] = mission_manager.active_map_id
                data["backend_status"] = mission_manager.state

                # Check for mission complete
                if data.get("mission", {}).get("status") == "COMPLETE":
                    if mission_manager.state == "RUNNING":
                        mission_manager.set_state("COMPLETE")

                out_msg = json.dumps(data)
                _LATEST_TELEMETRY = out_msg
                _TELEMETRY_RECEIVED = True
                _LAST_TIMESTAMP = msg_timestamp

                asyncio.run_coroutine_threadsafe(manager.broadcast(out_msg), self.loop)
        except json.JSONDecodeError:
            pass

import cv2  # type: ignore
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage

# Shared global buffers for camera streams
LATEST_FRAMES = {0: b"", 1: b""}
frame_lock = threading.Lock()

class CameraSubscriber(Node):
    def __init__(self, loop):
        super().__init__('camera_subscriber_backend')
        self.loop = loop
        self.bridge = CvBridge()
        self.create_subscription(RosImage, '/drone_0/camera/detection_image', lambda msg: self.img_callback(0, msg), 1)
        self.create_subscription(RosImage, '/drone_1/camera/detection_image', lambda msg: self.img_callback(1, msg), 1)

    def img_callback(self, drone_id, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            ret, buffer = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                with frame_lock:
                    LATEST_FRAMES[drone_id] = buffer.tobytes()
        except Exception as e:
            pass

def ros_spin_thread(loop):
    rclpy.init(args=None)
    telemetry_node = TelemetrySubscriber(loop)
    camera_node = CameraSubscriber(loop)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(telemetry_node)
    executor.add_node(camera_node)
    try:
        executor.spin()
    except Exception as e:
        print(f"ROS 2 subscriber thread exception: {e}")
    finally:
        telemetry_node.destroy_node()
        camera_node.destroy_node()
        rclpy.shutdown()

# ---------------------------------------------------------
# FastAPI App Lifecycle & Endpoints
# ---------------------------------------------------------
@app.on_event("startup")
async def startup_event():
    loop = asyncio.get_running_loop()
    threading.Thread(target=ros_spin_thread, args=(loop,), daemon=True).start()

@app.get("/api/health")
def health_check():
    return {
        "status": "ok",
        "ros_connected": rclpy.ok(),
        "telemetry_received": _TELEMETRY_RECEIVED,
        "connected_clients": len(manager.active_connections)
    }

@app.get("/api/maps")
def get_maps():
    return get_map_registry()

@app.get("/api/maps/{map_id}")
def get_world(map_id: str):
    return get_world_data(map_id)

class StartRequest(BaseModel):
    map_id: str
    drone_count: int = 2

@app.post("/api/mission/start")
async def start_mission(req: StartRequest):
    if req.map_id not in get_map_registry():
        raise HTTPException(status_code=400, detail="Invalid map_id")
    if req.drone_count < 1 or req.drone_count > 4:
        raise HTTPException(status_code=400, detail="drone_count must be 1–4")

    with mission_manager.lock:
        if mission_manager.state not in ["IDLE", "COMPLETE", "ERROR"]:
            raise HTTPException(status_code=400, detail="Mission already active or starting")

    # Fully clean up any previous completed/crashed mission state
    await mission_manager.stop_mission()
    global _LAST_TIMESTAMP
    _LAST_TIMESTAMP = None

    asyncio.create_task(start_mission_task(req.map_id, req.drone_count))
    return {"status": "starting", "map_id": req.map_id, "drone_count": req.drone_count}

@app.post("/api/mission/stop")
async def stop_mission():
    await mission_manager.stop_mission()
    global _LAST_TIMESTAMP
    _LAST_TIMESTAMP = None
    return {"status": "stopped"}

@app.post("/api/mission/reset")
async def reset_mission():
    await mission_manager.stop_mission()
    return {"status": "reset"}

@app.get("/api/mission/status")
def get_mission_status():
    return {
        "state": mission_manager.state,
        "active_map_id": mission_manager.active_map_id,
        "error": mission_manager.error_msg
    }

@app.post("/api/mission/view_simulation")
async def view_simulation():
    if mission_manager.state not in ["SIMULATOR_READY", "QMIX_STARTING", "RUNNING", "COMPLETE"]:
        raise HTTPException(status_code=400, detail="Simulation is not currently running")

    try:
        # Launch Gazebo GUI in the background attached to the running server session
        # We redirect output to /dev/null to prevent blocking
        subprocess.Popen(
            "export GZ_IP=127.0.0.1 && gz sim -g > /dev/null 2>&1",
            shell=True,
            preexec_fn=os.setsid
        )
        return {"status": "success", "message": "Gazebo GUI launched"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

async def mjpeg_generator(drone_id: int):
    while True:
        with frame_lock:
            frame = LATEST_FRAMES.get(drone_id)
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        await asyncio.sleep(0.05)  # 20 FPS max

from fastapi.responses import StreamingResponse

@app.get("/api/camera/stream")
async def camera_stream(drone_id: int):
    if drone_id not in [0, 1]:
        raise HTTPException(status_code=400, detail="Invalid drone ID")
    return StreamingResponse(mjpeg_generator(drone_id), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
