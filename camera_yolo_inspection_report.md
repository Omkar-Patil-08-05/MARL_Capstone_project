# CAMERA & YOLO PERCEPTION FORENSIC REPORT

## 1. CAMERA MODEL CURRENT STATE
**FILE:** `models/x500_mono_cam_down/model.sdf` and `model.config`
**RELEVANT CONTENT:** The model inherits from the base `x500`. It attaches a `camera_link` via a fixed joint pointing downwards (pitch 1.5707 rad). It uses the Gazebo Harmonic `gz-sim-sensors-system` plugin to simulate a camera at 1280x960 @ 30fps with a horizontal FOV of 1.74 rad.
**STATUS:** WORKING
**EVIDENCE:** The SDF is syntactically valid for Gazebo Harmonic. It explicitly declares `<sensor type="camera">` and uses Ogre2 rendering.

## 2. SIX-DRONE CAMERA ARCHITECTURE
**FILE:** `scripts/launch_swarm.sh` (Line 123)
**RELEVANT CONTENT:** The script currently exports `PX4_GZ_MODEL="x500"`.
**STATUS:** MISSING
**EVIDENCE:** The six-drone stack spawns the base `x500` model, which has absolutely no camera attached. The camera model `x500_mono_cam_down` is completely ignored at runtime.

## 3. ROS2 IMAGE PIPELINE CURRENT STATE
**FILE:** `drone_ws/src/swarm_controller/package.xml` and python environment.
**RELEVANT CONTENT:** Missing ROS2 package dependencies for image processing.
**STATUS:** BROKEN
**EVIDENCE:** 
- `ros2 pkg list` confirms `cv_bridge`, `sensor_msgs`, and `ros_gz_bridge` are installed on the system.
- However, `package.xml` does NOT declare `cv_bridge` or `sensor_msgs`, which is a package violation.
- Running `python3 -c "import ultralytics"` returns `ModuleNotFoundError: No module named 'ultralytics'`.

## 4. PREVIOUS CAMERA FAILURE — EXACT FINDINGS
**FILE:** `test_yolo.sh` and `yolo_human_detection.py`
**RELEVANT CONTENT:** The previous attempt used a mock `cam2image` ROS2 stream and an isolated `yolo_human_detection` node per drone.
**STATUS:** BROKEN
**EVIDENCE:** 
1. Gazebo never published camera frames because `x500` was used instead of `x500_mono_cam_down`.
2. Even if the camera existed, no `ros_gz_bridge` was configured to pipe the Gazebo topic into ROS2.
3. Because `ultralytics` was missing, the python node cleanly caught the `ImportError` and silently fell back to "MOCK" mode (drawing a hardcoded green bounding box in the center of the screen).

## 5. YOLO CURRENT STATE
**FILE:** `models/yolov8n.pt`
**RELEVANT CONTENT:** The standard pretrained YOLOv8 nano model (COCO dataset).
**STATUS:** WORKING (Asset exists) but MISSING (Library).
**EVIDENCE:** The `.pt` file exists on disk. However, the `ultralytics` library is missing from the Python environment, rendering the weights file unusable.

## 6. VICTIM ASSET CURRENT STATE
**FILE:** `assets_real/victims/rescue_randy_standing/model.sdf`
**RELEVANT CONTENT:** A low-poly human mannequin.
**STATUS:** WORKING
**EVIDENCE:** The model is spawned successfully by `world_generator/city.py`. It is physically present in Gazebo. However, there is a risk that a pretrained COCO YOLO model may fail to recognize a low-poly Gazebo mannequin as a real "person" (class 0).

## 7. CURRENT RUNTIME TOPICS
**FILE:** N/A (Runtime)
**RELEVANT CONTENT:** Minimum camera test environment.
**STATUS:** MISSING
**EVIDENCE:**
- Gazebo camera: NO (not spawned)
- ROS2 camera: NO (no bridge)
- Image frames: NO
- CameraInfo: NO
- OpenCV: NO
- YOLO: NO

## 8. DEPENDENCIES / GPU STATUS
**FILE:** N/A (System)
**RELEVANT CONTENT:** Hardware acceleration availability.
**STATUS:** WORKING
**EVIDENCE:**
- PyTorch: `2.2.2+cu121`
- CUDA: `True`
- GPU: `NVIDIA GeForce RTX 3050 6GB Laptop GPU`
- OpenCV: `4.6.0`
- Ultralytics: `MISSING`

## 9. ROOT CAUSE(S) OF PREVIOUS FAILURE
1. **Model Selection:** `scripts/launch_swarm.sh` hardcoded the `x500` model instead of `x500_mono_cam_down`.
2. **Missing Bridge:** No `ros_gz_bridge` node was launched to transport the Gazebo image topic to ROS2.
3. **Missing Dependency:** The `ultralytics` library was not installed, forcing the script into a silent mock mode.

## 10. WHAT IS ALREADY WORKING
- The physical `x500_mono_cam_down` model is correctly designed.
- PyTorch and CUDA are properly configured and detect the RTX 3050.
- The foundation for `yolo_human_detection.py` parsing bounding boxes into a `detection_data` JSON string is fundamentally correct.
- The dashboard is correctly parsing the `detection_data` payload.

## 11. WHAT IS MISSING
- `ultralytics` Python package.
- `ros_gz_bridge` routing commands in the launch scripts.
- Updating the drone model parameter in `launch_swarm.sh`.
- Proper ROS2 package dependencies.

## 12. RECOMMENDED CAMERA → YOLO ARCHITECTURE
Spawning six separate independent YOLO models (each consuming ~1GB of VRAM) on a 6GB RTX 3050 Laptop GPU while simultaneously rendering Gazebo Harmonic will likely cause an Out-Of-Memory (OOM) crash.
**Recommendation:** Refactor the perception pipeline into a **single centralized node** (`swarm_perception_node.py`). This node should subscribe to all six `/drone_X/camera/image_raw` topics concurrently, load exactly **one** instance of YOLOv8n into VRAM, and run batched or sequential inference.

## 13. FILES THAT WILL NEED MODIFICATION
- `scripts/launch_swarm.sh`
- `drone_ws/src/swarm_controller/package.xml`
- `drone_ws/src/swarm_controller/setup.py`
- `drone_ws/src/swarm_controller/swarm_controller/yolo_human_detection.py` (needs major refactoring for central handling).

## 14. FILES THAT MUST NOT BE MODIFIED
- `drone_ws/src/swarm_controller/swarm_controller/qmix_drone_test.py`
- `scripts/launch_6_drone_px4.sh`
- Dashboard React code.

## 15. STEP-BY-STEP IMPLEMENTATION PLAN
1. Run `pip install ultralytics` to resolve the missing dependency.
2. Edit `scripts/launch_swarm.sh` to export `PX4_GZ_MODEL="x500_mono_cam_down"`.
3. Add `ros2 run ros_gz_bridge parameter_bridge` commands inside `launch_swarm.sh` to route `/camera@sensor_msgs/msg/Image@gz.msgs.Image` to `/drone_X/camera/image_raw`.
4. Delete the six independent YOLO calls at the bottom of `launch_swarm.sh`.
5. Create a new `swarm_perception_node.py` in `swarm_controller` that listens to all 6 topics and uses a single YOLO instance.
6. Run `colcon build`.
7. Launch the system.

## 16. EXACT COMMANDS TO TEST IT
```bash
pip install ultralytics
# After modifications and colcon build:
bash scripts/launch_6_drone_px4.sh
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /drone_0/camera/detection_data
```

## 17. RISKS / PERFORMANCE CONCERNS
- **Rendering Cost:** Simulating 6 distinct 1280x960 30FPS cameras in Gazebo is extremely taxing. If Gazebo real-time factor (RTF) drops below 0.3, PX4 will desync. Consider reducing the camera resolution in `model.sdf` to 640x480 and update rate to 10 FPS.
- **Domain Shift:** YOLOv8 COCO is trained on real human photos. It may not confidently recognize the Gazebo `rescue_randy_standing` mannequin. Threshold tuning (`conf > 0.2`) or custom training may be required.
