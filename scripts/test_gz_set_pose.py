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

    print("[2] Repositioning camera to z=12m via gz service...")
    pose_req = 'name: "camera_drone", position: {x: 0, y: 0, z: 12.0}, orientation: {x: 0, y: 0.7071068, z: 0, w: 0.7071068}'
    cmd_set = f"gz service -s /world/test_camera_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req '{pose_req}'"
    res = subprocess.run(cmd_set, shell=True, capture_output=True, text=True)
    print("set_pose result:", res.stdout.strip(), res.stderr.strip())

    print("[3] Repositioning camera to x=10m, y=10m via gz service...")
    pose_req2 = 'name: "camera_drone", position: {x: 10, y: 10, z: 8.0}, orientation: {x: 0, y: 0.7071068, z: 0, w: 0.7071068}'
    cmd_set2 = f"gz service -s /world/test_camera_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req '{pose_req2}'"
    res2 = subprocess.run(cmd_set2, shell=True, capture_output=True, text=True)
    print("set_pose result 2:", res2.stdout.strip(), res2.stderr.strip())

finally:
    cleanup()
    print("Done.")
