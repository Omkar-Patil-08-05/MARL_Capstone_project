#!/bin/bash
# ==============================================================================
# Single-Drone Forest Simulation & Flight Readiness Launcher (Phase F5.3D)
# ==============================================================================
# - Headless, time-limited, staged execution preserving protected baselines.
# - Reads spawn coordinates dynamically from metadata (terrain-aware).
# - Validates thermal camera output via Gazebo Harmonic and ROS 2 Jazzy.
# - Clean process management: scoped cleanup with fail-closed teardown guard.
# - Teardown guard strictly queries fresh authoritative ROS 2 telemetry
#   (arming_state, landed, ground_contact, resting velocity) rather than log grepping.
# - All termination paths (SIGINT, SIGTERM, watchdog, unexpected exit) fail closed.
# ==============================================================================
set -e

WORKSPACE_DIR="/home/capstone/capstone_project_antigravity"
META_FILE="$WORKSPACE_DIR/generated_worlds/forest_stage1_baseline_metadata.json"
WORLD_FILE="$WORKSPACE_DIR/generated_worlds/forest_stage1_baseline.sdf"
WORLD_NAME=$(python3 -c "
import xml.etree.ElementTree as ET
tree = ET.parse('$WORLD_FILE')
world = tree.getroot().find('world')
print(world.attrib['name'] if world is not None and 'name' in world.attrib else 'realistic_forest_sar')
")
LOG_DIR="/tmp/test_forest_one_drone"

mkdir -p "$LOG_DIR"

MODE="sensor-only"
TIMEOUT_SEC=60
ARMING_REQUESTED=0
INDUCE_TEARDOWN_REFUSAL=0

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --with-px4) MODE="with-px4" ;;
        --sensor-only) MODE="sensor-only" ;;
        --controller-dry-run) MODE="controller-dry-run" ;;
        --test-offboard-loss) MODE="test-offboard-loss"; ARMING_REQUESTED=1 ;;
        --induce-teardown-refusal) MODE="induce-teardown-refusal"; ARMING_REQUESTED=1; INDUCE_TEARDOWN_REFUSAL=1 ;;
        --test-termination-path) MODE="test-termination-path:$2"; ARMING_REQUESTED=1; shift ;;
        --watchdog-test) MODE="watchdog-test" ;;
        --timeout) TIMEOUT_SEC="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

echo "======================================================================"
echo "Phase F5.3D: Single Drone Forest Simulation & Readiness Test"
echo "Mode:             $MODE"
echo "World:            $WORLD_NAME"
echo "Timeout:          ${TIMEOUT_SEC}s"
echo "Arming Requested: $ARMING_REQUESTED"
echo "======================================================================"

# Track local PIDs for scoped cleanup
MAIN_PID=$$
GZ_PID=""
AGENT_PID=""
PX4_PID=""
BRIDGE_PID=""
TEST_PID=""
WATCHDOG_PID=""
EXIT_CODE=0
CLEANUP_DONE=0
CLEANUP_INHIBITED=0
SAFETY_CONFIRMED=0

cleanup() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return
    fi
    if [ "$CLEANUP_INHIBITED" -eq 1 ]; then
        echo "======================================================================" >&2
        echo "[Cleanup Guard] REFUSED: Simulator teardown is INHIBITED because" >&2
        echo "vehicle landing and disarm were NOT positively confirmed! Preserving" >&2
        echo "simulator processes (Gazebo PID: $GZ_PID, PX4 PID: $PX4_PID) to avoid" >&2
        echo "mid-air cutoff." >&2
        echo "======================================================================" >&2
        return
    fi
    CLEANUP_DONE=1

    echo ""
    echo "[Cleanup] Stopping test processes..."
    if [ -n "$WATCHDOG_PID" ] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
        kill "$WATCHDOG_PID" 2>/dev/null || true
    fi
    if [ -n "$TEST_PID" ] && kill -0 "$TEST_PID" 2>/dev/null; then
        echo "[Cleanup] Stopping test node (PID: $TEST_PID)..."
        kill "$TEST_PID" 2>/dev/null || true
    fi
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "[Cleanup] Stopping ros_gz_bridge (PID: $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null || true
    fi
    if [ -n "$PX4_PID" ] && kill -0 "$PX4_PID" 2>/dev/null; then
        echo "[Cleanup] Stopping PX4 instance (PID: $PX4_PID)..."
        kill "$PX4_PID" 2>/dev/null || true
    fi
    if [ -n "$AGENT_PID" ] && kill -0 "$AGENT_PID" 2>/dev/null; then
        echo "[Cleanup] Stopping MicroXRCEAgent (PID: $AGENT_PID)..."
        kill "$AGENT_PID" 2>/dev/null || true
    fi
    if [ -n "$GZ_PID" ] && kill -0 "$GZ_PID" 2>/dev/null; then
        echo "[Cleanup] Stopping Gazebo server (PID: $GZ_PID)..."
        kill "$GZ_PID" 2>/dev/null || true
    fi
    
    # Grace period before SIGKILL
    sleep 0.5
    for pid_var in "$TEST_PID" "$BRIDGE_PID" "$PX4_PID" "$AGENT_PID" "$GZ_PID"; do
        if [ -n "$pid_var" ] && kill -0 "$pid_var" 2>/dev/null; then
            kill -9 "$pid_var" 2>/dev/null || true
        fi
    done
    echo "[Cleanup] Done."
}

# Unified Authoritative Teardown Guard (FAIL-CLOSED)
# Required across ALL termination paths whenever PX4 is active.
verify_safety_before_cleanup() {
    local trigger="$1"

    if [ "$CLEANUP_DONE" -eq 1 ] || [ "$CLEANUP_INHIBITED" -eq 1 ]; then
        return
    fi

    # Review of PX4 process absence/loss (Finding D):
    # If arming was requested, absence or death of PX4 must NOT be assumed safe.
    if [ "$ARMING_REQUESTED" -eq 1 ]; then
        if [ -z "$PX4_PID" ] || ! kill -0 "$PX4_PID" 2>/dev/null; then
            echo "======================================================================" >&2
            echo "[Safety Guard] CRITICAL SAFETY FAULT: PX4 process is absent or dead while ARMING_REQUESTED=1!" >&2
            echo "Trigger: $trigger" >&2
            echo "Vehicle state cannot be authoritatively verified safe when PX4 is dead." >&2
            echo "Refusing simulator teardown to preserve post-mortem evidence." >&2
            echo "Preserving Gazebo (PID: $GZ_PID)." >&2
            echo "Reporting unresolved safety state explicitly in $LOG_DIR/unresolved_safety_state.log" >&2
            echo "======================================================================" >&2
            echo "UNRESOLVED_SAFETY_STATE: trigger=$trigger, reason=PX4_PROCESS_ABSENT_OR_DEAD, arming_requested=$ARMING_REQUESTED" > "$LOG_DIR/unresolved_safety_state.log"
            CLEANUP_INHIBITED=1
            EXIT_CODE=101
            exit 101
        fi
    elif [ -z "$PX4_PID" ] || ! kill -0 "$PX4_PID" 2>/dev/null; then
        # If arming was never requested and PX4 is not running, no armed vehicle was possible.
        return
    fi

    echo ""
    echo "[Safety Guard] Verifying authoritative vehicle state prior to teardown (Trigger: $trigger)..."

    local is_safe=0
    local check_rc=4
    local max_attempts=1

    # In post-test or watchdog timeout, allow grace period (up to 30s) if vehicle is descending/disarming
    if [ "$trigger" == "POST_TEST" ] || [ "$trigger" == "SIGTERM_WATCHDOG" ]; then
        if [[ "$MODE" =~ ^test-termination-path: ]] || [ "$INDUCE_TEARDOWN_REFUSAL" -eq 1 ]; then
            max_attempts=1
        else
            max_attempts=15
        fi
    fi

    local v_timeout=4.0
    if [[ "$MODE" =~ ^test-termination-path: ]] || [ "$INDUCE_TEARDOWN_REFUSAL" -eq 1 ]; then
        v_timeout=1.0
    fi

    local v_extra_args=()
    if [ "$INDUCE_TEARDOWN_REFUSAL" -eq 1 ]; then
        # Controlled verifier failure injection through the production guard function (P1 Resolution):
        # Enforce an impossible max-speed threshold (-1.0 m/s) to trigger authoritative rejection (rc=3)
        v_extra_args+=(--max-speed -1.0)
    fi

    for ((attempt=1; attempt<=max_attempts; attempt++)); do
        set +e
        python3 "$WORKSPACE_DIR/scripts/verify_vehicle_state_teardown.py" \
            --namespace "drone_0" \
            "${v_extra_args[@]}" \
            --timeout "$v_timeout" > "$LOG_DIR/authoritative_teardown_check.log" 2>&1
        check_rc=$?
        set -e

        if [ "$check_rc" -eq 0 ]; then
            echo "[Safety Guard] Authoritative state positively confirmed: vehicle is DISARMED and safely on ground."
            cat "$LOG_DIR/authoritative_teardown_check.log"
            is_safe=1
            SAFETY_CONFIRMED=1
            break
        elif [ "$check_rc" -eq 2 ]; then
            echo "[Safety Guard] Attempt $attempt/$max_attempts: Vehicle is ARMED. Waiting for autonomous disarm..."
            sleep 2
        elif [ "$check_rc" -eq 3 ]; then
            echo "[Safety Guard] Attempt $attempt/$max_attempts: Vehicle is AIRBORNE/IN-MOTION. Waiting for landing..."
            sleep 2
        else
            echo "[Safety Guard] Attempt $attempt/$max_attempts: Telemetry query unconfirmed (rc=$check_rc)..."
            sleep 1
        fi
    done

    if [ "$is_safe" -ne 1 ]; then
        echo "======================================================================" >&2
        echo "[Safety Guard] CRITICAL SAFETY FAULT: Fail-Closed Protection Tripped!" >&2
        echo "Trigger: $trigger" >&2
        echo "Vehicle landing and disarmed state could NOT be authoritatively verified!" >&2
        echo "Authoritative check exited with code $check_rc." >&2
        cat "$LOG_DIR/authoritative_teardown_check.log" >&2
        echo "Refusing simulator teardown to prevent mid-air process cutoff." >&2
        echo "Preserving Gazebo (PID: $GZ_PID) and PX4 (PID: $PX4_PID)." >&2
        echo "Reporting unresolved safety state explicitly in $LOG_DIR/unresolved_safety_state.log" >&2
        echo "======================================================================" >&2
        echo "UNRESOLVED_SAFETY_STATE: trigger=$trigger, check_rc=$check_rc, arming_requested=$ARMING_REQUESTED" > "$LOG_DIR/unresolved_safety_state.log"
        CLEANUP_INHIBITED=1
        EXIT_CODE=101
        exit 101
    fi
}

on_sigterm() {
    echo ""
    echo "[Signal Handler] Caught SIGTERM (Watchdog timeout or external abort)."
    EXIT_CODE=124
    verify_safety_before_cleanup "SIGTERM_WATCHDOG"
    cleanup
    exit "$EXIT_CODE"
}

on_sigint() {
    echo ""
    echo "[Signal Handler] Caught SIGINT (Ctrl+C). Terminating with failure."
    EXIT_CODE=130
    verify_safety_before_cleanup "SIGINT_CTRL_C"
    cleanup
    exit "$EXIT_CODE"
}

on_exit() {
    local rc=$?
    if [ "$CLEANUP_DONE" -eq 0 ] && [ "$CLEANUP_INHIBITED" -eq 0 ]; then
        if [ "$rc" -ne 0 ] && [ "$EXIT_CODE" -eq 0 ]; then
            EXIT_CODE="$rc"
        fi
        if [ "$SAFETY_CONFIRMED" -eq 0 ]; then
            verify_safety_before_cleanup "EXIT_TRAP"
        fi
        cleanup
    fi
    if [ "$EXIT_CODE" -ne 0 ]; then
        exit "$EXIT_CODE"
    fi
    exit "$rc"
}

trap on_sigterm TERM
trap on_sigint INT
trap on_exit EXIT

# Start global runtime watchdog to strictly bound launcher execution
(
    sleep "$TIMEOUT_SEC"
    echo ""
    echo "[Watchdog] Script exceeded total timeout of ${TIMEOUT_SEC}s! Forcing termination of PID $MAIN_PID."
    kill -TERM "$MAIN_PID" 2>/dev/null || true
) &
WATCHDOG_PID=$!

# Handle dedicated termination-path test modes directly in the production launcher
if [[ "$MODE" =~ ^test-termination-path: ]]; then
    TEST_PATH="${MODE#test-termination-path:}"
    echo "[Termination Path Test] Exercising production launcher under scenario: $TEST_PATH..."
    
    # Spawn dummy background processes to represent Gazebo and PX4
    sleep 300 &
    GZ_PID=$!
    echo "$GZ_PID" > "$LOG_DIR/tracked_gz_pid.txt"
    
    if [ "$TEST_PATH" != "px4-dead" ]; then
        sleep 300 &
        PX4_PID=$!
        echo "$PX4_PID" > "$LOG_DIR/tracked_px4_pid.txt"
    else
        PX4_PID=""
    fi
    
    case "$TEST_PATH" in
        post-test-verifier-fail)
            # Post-test with unverified state (no telemetry publisher -> timeout rc=4)
            verify_safety_before_cleanup "POST_TEST"
            cleanup
            ;;
        sigint)
            # Send SIGINT to self
            kill -INT "$MAIN_PID"
            sleep 1
            ;;
        sigterm)
            # Send SIGTERM to self
            kill -TERM "$MAIN_PID"
            sleep 1
            ;;
        crash)
            # Unexpected script failure triggering on_exit trap
            exit 1
            ;;
        px4-dead)
            # PX4 absent/dead while arming was requested
            verify_safety_before_cleanup "POST_TEST"
            cleanup
            ;;
        *)
            echo "Unknown termination test path: $TEST_PATH"
            exit 1
            ;;
    esac
    exit 0
fi

# 1. Environment Setup
echo "[1/6] Sourcing environments..."
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

source "$WORKSPACE_DIR/scripts/setup_env.sh"
source "/opt/ros/jazzy/setup.bash"
source "$WORKSPACE_DIR/drone_ws/install/setup.bash" 2>/dev/null || true
source "/home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"

export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/models:$WORKSPACE_DIR/assets_real/victims:${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"

# 2. Extract Terrain-Aware Spawn Coordinates
echo "[2/6] Reading spawn coordinates from staged metadata..."
if [ ! -f "$META_FILE" ]; then
    echo "Error: Metadata file missing at $META_FILE"
    exit 1
fi

D0_X=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][0]['x'])")
D0_Y=$(python3 -c "import json; print(json.load(open('$META_FILE'))['drone_base']['spawns'][0]['y'])")
D0_Z=$(python3 -c "import json; print(float(json.load(open('$META_FILE'))['drone_base']['spawns'][0]['z']) + 0.35)")

echo "  Drone 0 Spawn: X=$D0_X, Y=$D0_Y, Z=$D0_Z (Terrain-aware altitude)"

# 3. Prepare Staged World with Drone Included (Preserving Original Baseline)
RUN_WORLD_FILE="$LOG_DIR/stage1_with_thermal_drone.sdf"
python3 "$WORKSPACE_DIR/scripts/stage_world.py" \
    --base-world "$WORLD_FILE" \
    --output-world "$RUN_WORLD_FILE" \
    --drone-x "$D0_X" \
    --drone-y "$D0_Y" \
    --drone-z "$D0_Z" \
    --log-dir "$LOG_DIR"

echo "  Validating staged SDF syntax with Gazebo..."
gz sdf -k "$RUN_WORLD_FILE" > "$LOG_DIR/sdf_validation.log" 2>&1 || {
    echo "Error: Staged SDF validation failed!"
    cat "$LOG_DIR/sdf_validation.log"
    exit 1
}
echo "  Staged SDF validation: PASSED."

# Verify spherical coordinates injection in staged SDF
if grep -q "<spherical_coordinates>" "$RUN_WORLD_FILE"; then
    echo "  Staged SDF spherical coordinates verified: $(grep -o "<latitude_deg>[^<]*" "$RUN_WORLD_FILE"), $(grep -o "<longitude_deg>[^<]*" "$RUN_WORLD_FILE")"
else
    echo "Error: Missing spherical coordinates in staged SDF!" >&2
    exit 1
fi

# 4. Start Gazebo Server Headless
echo "[3/6] Starting Gazebo server headless on staged world..."
gz sim -s -v 2 -r "$RUN_WORLD_FILE" > "$LOG_DIR/gz.log" 2>&1 &
GZ_PID=$!
echo "$GZ_PID" > "$LOG_DIR/tracked_gz_pid.txt"
echo "  Gazebo PID: $GZ_PID"

# Wait for simulation clock
echo "  Waiting for Gazebo simulation clock (/clock)..."
CLOCK_ACTIVE=0
for i in {1..30}; do
    if gz topic -e -t /clock -n 1 > /dev/null 2>&1; then
        CLOCK_ACTIVE=1
        break
    fi
    sleep 1
done

if [ $CLOCK_ACTIVE -eq 0 ]; then
    echo "Error: Timed out waiting for Gazebo simulation clock!"
    exit 1
fi
echo "  Simulation clock active."
sleep 5

# 5. Model Verification and Optional PX4 Connection
echo "[4/6] Verifying x500_thermal_0 model and sensor..."
sleep 2

# Verify model existence in Gazebo
echo "  Verifying spawned models in Gazebo:"
gz model --list | grep x500_thermal || echo "Warning: x500_thermal not found in model list yet."

# Start ROS 2 Gazebo Bridge for Thermal Topic and Clock
echo "[5/6] Starting ros_gz_bridge for thermal camera and clock..."
ros2 run ros_gz_bridge parameter_bridge \
    /drone_0/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > "$LOG_DIR/tracked_bridge_pid.txt"
echo "  Bridge PID: $BRIDGE_PID"
sleep 3

if [ "$MODE" == "with-px4" ] || [ "$MODE" == "controller-dry-run" ] || [ "$MODE" == "test-offboard-loss" ] || [ "$MODE" == "induce-teardown-refusal" ]; then
    # PX4 mode: check MicroXRCEAgent on port 8888
    if ss -ulnp | grep -q ":8888 "; then
        echo "  MicroXRCEAgent already active on UDP 8888, reusing."
    else
        echo "  Starting MicroXRCEAgent on UDP 8888..."
        MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
        AGENT_PID=$!
        echo "$AGENT_PID" > "$LOG_DIR/tracked_agent_pid.txt"
        sleep 2
    fi

    echo "  Starting PX4 SITL Instance 0 (authoritative attach to x500_thermal_0)..."
    PX4_BUILD="/home/capstone/PX4-Autopilot/build/px4_sitl_default"
    INSTANCE_DIR="/tmp/px4_instance_forest_0"
    rm -rf "$INSTANCE_DIR" && mkdir -p "$INSTANCE_DIR"
    (
        cd "$INSTANCE_DIR"
        export PX4_GZ_STANDALONE=1
        export PX4_PARAM_NAV_DLL_ACT=0
        export PX4_PARAM_COM_OBL_RC_ACT=4   # 4 = Land mode upon loss of offboard (ensures safe descent)
        export PX4_PARAM_COM_OF_LOSS_T=1.0  # 1.0s timeout before offboard loss failsafe triggers
        export PX4_PARAM_CBRK_SUPPLY_CHK=894281  # Disable power check for simulated SITL
        export PX4_PARAM_COM_ARM_MAG_STR=0       # Disable mag interference arm check for simulated SITL
        export PX4_SYS_AUTOSTART=4001
        export PX4_SIMULATOR=gz
        export PX4_GZ_WORLD="$WORLD_NAME"
        export PX4_GZ_MODEL_NAME="x500_thermal_0"
        export PX4_UXRCE_DDS_NS="drone_0"
        exec "$PX4_BUILD/bin/px4" -i 0 -d "$PX4_BUILD/etc" > "$LOG_DIR/px4_out.log" 2> "$LOG_DIR/px4_err.log"
    ) &
    PX4_PID=$!
    echo "$PX4_PID" > "$LOG_DIR/tracked_px4_pid.txt"
    echo "  PX4 PID: $PX4_PID (Waiting 10s for EKF stabilization)..."
    sleep 10

    # Read back and verify active failsafe parameters
    echo "  Checking PX4 failsafe parameter updates in runtime log:"
    grep -E "COM_OBL_RC_ACT|COM_OF_LOSS_T" "$LOG_DIR/px4_out.log" || echo "  (Parameters loaded via environment defaults)"
    echo "  Direct query of active runtime failsafe parameters via px4-param:"
    rm -f "$LOG_DIR/param_readback.log"
    if ! "$PX4_BUILD/bin/px4-param" --instance 0 show COM_OBL_RC_ACT > "$LOG_DIR/param_readback.log" 2>&1; then
        echo "Error: Mandatory failsafe parameter query failed for COM_OBL_RC_ACT!" >&2
        cat "$LOG_DIR/param_readback.log" >&2
        exit 1
    fi
    if ! "$PX4_BUILD/bin/px4-param" --instance 0 show COM_OF_LOSS_T >> "$LOG_DIR/param_readback.log" 2>&1; then
        echo "Error: Mandatory failsafe parameter query failed for COM_OF_LOSS_T!" >&2
        cat "$LOG_DIR/param_readback.log" >&2
        exit 1
    fi
    cat "$LOG_DIR/param_readback.log"

    ACT_VAL=$(grep "COM_OBL_RC_ACT" "$LOG_DIR/param_readback.log" | awk -F':' '{print $NF}' | tr -d '[:space:]')
    LOSS_T_VAL=$(grep "COM_OF_LOSS_T" "$LOG_DIR/param_readback.log" | awk -F':' '{print $NF}' | tr -d '[:space:]')

    echo "  Failsafe Parameter Enforcement: COM_OBL_RC_ACT='$ACT_VAL', COM_OF_LOSS_T='$LOSS_T_VAL'"
    if [ "$ACT_VAL" != "4" ]; then
        echo "Error: Mandatory failsafe check failed for COM_OBL_RC_ACT! Got '$ACT_VAL', expected '4'." >&2
        exit 1
    fi
    if [ "$LOSS_T_VAL" != "1.0000" ]; then
        echo "Error: Mandatory failsafe check failed for COM_OF_LOSS_T! Got '$LOSS_T_VAL', expected '1.0000'." >&2
        exit 1
    fi
    echo "  Mandatory failsafe parameters verified and ENFORCED: COM_OBL_RC_ACT=4, COM_OF_LOSS_T=1.0000."

    if [ "$MODE" == "controller-dry-run" ]; then
        # 6. Run Controller State Transitions with Flight Authorization Disabled
        echo "[6/6] Executing flight controller state machine in non-arming dry-run mode..."
        python3 "$WORKSPACE_DIR/drone_ws/src/single_drone_test/single_drone_test/flight_test.py" \
            --namespace "drone_0" \
            --system-id 1 \
            --takeoff-altitude 5.0 \
            --hold-time 5.0 \
            --vertical-only > >(tee "$LOG_DIR/controller.log") 2>&1 &
        TEST_PID=$!
        set +e
        wait "$TEST_PID"
        TEST_RC=$?
        set -e
        TEST_PID=""

        if [ "$TEST_RC" -ne 0 ]; then
            echo "[Safety Watchdog] Controller exited with code $TEST_RC."
            EXIT_CODE="$TEST_RC"
        fi
    elif [ "$MODE" == "test-offboard-loss" ]; then
        # 6. Run Controlled Offboard-Loss Fault Injection and Recovery Test
        echo "[6/6] Executing controlled offboard-loss fault injection and failsafe recovery test..."
        python3 "$WORKSPACE_DIR/scripts/test_offboard_loss_failsafe.py" \
            --namespace "drone_0" \
            --system-id 1 \
            --timeout 45.0 > >(tee "$LOG_DIR/offboard_loss_test.log") 2>&1 &
        TEST_PID=$!
        set +e
        wait "$TEST_PID"
        TEST_RC=$?
        set -e
        TEST_PID=""
        if [ "$TEST_RC" -ne 0 ]; then
            echo "Error: Offboard loss recovery test failed with exit code $TEST_RC!" >&2
            EXIT_CODE="$TEST_RC"
        fi
    elif [ "$MODE" == "induce-teardown-refusal" ]; then
        echo "[6/6] Inducing teardown refusal on live Gazebo & PX4 processes..."
        echo "  Exercising production safety guard (verify_safety_before_cleanup) via controlled verifier failure..."
        INDUCE_TEARDOWN_REFUSAL=1
        ARMING_REQUESTED=1
    elif [ "$MODE" != "controller-dry-run" ] && [ "$MODE" != "test-offboard-loss" ]; then
        # 6. Run PX4 Non-Flight Readiness Verification
        echo "[6/6] Executing PX4 non-flight readiness verification..."
        python3 "$WORKSPACE_DIR/scripts/verify_px4_readiness.py" \
            --namespace "drone_0" \
            --samples 15 \
            --timeout 25.0
    fi

    # Post-controller authoritative vehicle state verification (FAIL-CLOSED)
    verify_safety_before_cleanup "POST_TEST"

else
    # 6. Run Strict Thermal Sensor Verification
    echo "[6/6] Executing rigorous thermal sensor verification..."
    python3 "$WORKSPACE_DIR/scripts/verify_thermal_sensor.py" \
        --topic "/drone_0/thermal/image_raw" \
        --frames 10 \
        --timeout 20.0 \
        --expected-width 320 \
        --expected-height 240 \
        --expected-encoding mono16 \
        --output "$LOG_DIR/thermal_frame_sample.png"
fi

echo "Test run complete."
