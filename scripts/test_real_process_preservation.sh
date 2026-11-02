#!/bin/bash
# ==============================================================================
# Real-Process Preservation Integration Test (Finding D & P1)
# ==============================================================================
# Demonstrates that a live, running Gazebo Harmonic instance and PX4 SITL
# instance survive an inhibited teardown check executed through the actual
# production launcher (scripts/test_forest_one_drone.sh):
# 1. Runs the production launcher under --induce-teardown-refusal.
# 2. Confirms production launcher trips fail-closed protection and exits with code 101.
# 3. Reads tracked PIDs directly from production launcher logs.
# 4. Confirms live Gazebo process survived, matches cmdline 'gz sim', and is not a zombie.
# 5. Confirms no other Gazebo instances are running on the host system.
# 6. Samples simulation clock (/clock) across distinct wall-clock times to confirm
#    simulation time is actively advancing (T2 > T1), proving healthy simulation.
# 7. Confirms live PX4 process survived, matches cmdline 'px4', and is not a zombie.
# 8. Cleanly terminates the preserved processes post-verification.
# ==============================================================================
set -e

WORKSPACE_DIR="/home/capstone/capstone_project_antigravity"
LAUNCHER="$WORKSPACE_DIR/scripts/test_forest_one_drone.sh"
LOG_DIR="/tmp/test_forest_one_drone"

echo "======================================================================"
echo "RUNNING REAL-PROCESS PRESERVATION INTEGRATION TEST (FINDING P1)"
echo "Target: Live Gazebo Harmonic & PX4 SITL Survival via Production Launcher"
echo "Launcher: $LAUNCHER"
echo "======================================================================"

# Helper function: recursively retrieve all descendants of a tracked PID (Issue A Resolution)
get_all_descendants() {
    local parent_pid="$1"
    local children
    children=$(pgrep -P "$parent_pid" 2>/dev/null || true)
    for child in $children; do
        get_all_descendants "$child"
        echo "$child"
    done
}

# Helper function: strictly scoped process tree termination with identity verification (P0 / Issue A Resolution)
terminate_scoped_process_tree() {
    local pid="$1"
    local expected_pattern="$2"
    local desc="$3"

    if [ -z "$pid" ]; then
        return 0
    fi

    if kill -0 "$pid" 2>/dev/null; then
        local cmdline=""
        if [ -f "/proc/$pid/cmdline" ]; then
            cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
        fi
        if [[ "$cmdline" =~ $expected_pattern ]]; then
            # Find all descendants belonging to this tracked process hierarchy
            local descendants=()
            while IFS= read -r dpid; do
                if [ -n "$dpid" ]; then
                    descendants+=("$dpid")
                fi
            done < <(get_all_descendants "$pid")

            echo "  Terminating verified $desc (PID: $pid) and ${#descendants[@]} descendant(s)..."

            # 1. Gracefully terminate descendants then parent with SIGTERM
            for d in "${descendants[@]}"; do
                kill -TERM "$d" 2>/dev/null || true
            done
            kill -TERM "$pid" 2>/dev/null || true

            # Grace period up to 1.0s
            for i in {1..10}; do
                local any_alive=0
                if kill -0 "$pid" 2>/dev/null; then any_alive=1; fi
                for d in "${descendants[@]}"; do
                    if kill -0 "$d" 2>/dev/null; then any_alive=1; fi
                done
                if [ "$any_alive" -eq 0 ]; then
                    break
                fi
                sleep 0.1
            done

            # 2. Force terminate with SIGKILL if still present
            for d in "${descendants[@]}"; do
                if kill -0 "$d" 2>/dev/null; then
                    kill -9 "$d" 2>/dev/null || true
                fi
            done
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null || true
            fi
        else
            echo "  Warning: PID $pid does not match identity '$expected_pattern' (cmd: '$cmdline'), skipping termination."
        fi
    fi
}

CLEANUP_DONE=0
cleanup_all_tracked() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return 0
    fi
    CLEANUP_DONE=1
    echo "Executing failure-safe cleanup of tracked test processes and descendants..."
    if [ -f "$LOG_DIR/tracked_px4_pid.txt" ]; then
        local p
        p=$(cat "$LOG_DIR/tracked_px4_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$p" "px4" "Tracked PX4"
    fi
    if [ -f "$LOG_DIR/tracked_gz_pid.txt" ]; then
        local g
        g=$(cat "$LOG_DIR/tracked_gz_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$g" "gz[[:space:]]+sim" "Tracked Gazebo"
    fi
    if [ -f "$LOG_DIR/tracked_bridge_pid.txt" ]; then
        local b
        b=$(cat "$LOG_DIR/tracked_bridge_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$b" "ros_gz_bridge" "Tracked Bridge"
    fi
    if [ -f "$LOG_DIR/tracked_agent_pid.txt" ]; then
        local a
        a=$(cat "$LOG_DIR/tracked_agent_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$a" "MicroXRCEAgent" "Tracked Agent"
    fi
}

on_exit() {
    local rc=$?
    if [ "$CLEANUP_DONE" -eq 0 ]; then
        echo ""
        echo "[Exit Trap] Script exiting (code: $rc). Running failure-safe tracked process cleanup..."
        cleanup_all_tracked
    fi
    exit "$rc"
}

trap on_exit EXIT
trap on_exit TERM
trap on_exit INT

# 0. Pre-test sanitation: clean up only leftover tracked processes from previous runs of this test (P0 Resolution)
echo "[0/4] Checking for previous tracked test processes..."
if [ -d "$LOG_DIR" ]; then
    if [ -f "$LOG_DIR/tracked_px4_pid.txt" ]; then
        OLD_PX4=$(cat "$LOG_DIR/tracked_px4_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$OLD_PX4" "px4" "Previous PX4"
    fi
    if [ -f "$LOG_DIR/tracked_gz_pid.txt" ]; then
        OLD_GZ=$(cat "$LOG_DIR/tracked_gz_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$OLD_GZ" "gz[[:space:]]+sim" "Previous Gazebo"
    fi
    if [ -f "$LOG_DIR/tracked_bridge_pid.txt" ]; then
        OLD_BRIDGE=$(cat "$LOG_DIR/tracked_bridge_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$OLD_BRIDGE" "ros_gz_bridge" "Previous Bridge"
    fi
    if [ -f "$LOG_DIR/tracked_agent_pid.txt" ]; then
        OLD_AGENT=$(cat "$LOG_DIR/tracked_agent_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_scoped_process_tree "$OLD_AGENT" "MicroXRCEAgent" "Previous Agent"
    fi
fi
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"

# 1. Execute production launcher with induced teardown refusal
echo "[1/4] Executing production launcher with induced teardown refusal..."
set +e
"$LAUNCHER" --induce-teardown-refusal --timeout 45 > "$LOG_DIR/launcher_run.log" 2>&1
LAUNCHER_RC=$?
set -e

echo "  Production Launcher Exit Code: $LAUNCHER_RC (Expected: 101)"
if [ "$LAUNCHER_RC" -ne 101 ]; then
    echo "Error: Production launcher did not exit with code 101!" >&2
    cat "$LOG_DIR/launcher_run.log" >&2
    exit 1
fi

# 2. Retrieve tracked PIDs from production launcher
echo "[2/4] Reading tracked simulator PIDs from production launcher logs..."
if [ ! -f "$LOG_DIR/tracked_gz_pid.txt" ] || [ ! -f "$LOG_DIR/tracked_px4_pid.txt" ]; then
    echo "Error: Tracked PID files missing from launcher run!" >&2
    exit 1
fi

GZ_PID=$(cat "$LOG_DIR/tracked_gz_pid.txt" | tr -d '[:space:]')
PX4_PID=$(cat "$LOG_DIR/tracked_px4_pid.txt" | tr -d '[:space:]')
echo "  Tracked Gazebo PID: $GZ_PID"
echo "  Tracked PX4 PID:    $PX4_PID"

# 3. Verify fail-closed log recorded refusal
if [ ! -f "$LOG_DIR/unresolved_safety_state.log" ]; then
    echo "Error: Unresolved safety state log missing!" >&2
    exit 1
fi
echo "  Logged State: $(cat "$LOG_DIR/unresolved_safety_state.log")"

# 4. Critical Post-Teardown Process & Simulation Health Assertions
echo "----------------------------------------------------------------------"
echo "[3/4] Asserting Gazebo process health and isolating instance..."
echo "----------------------------------------------------------------------"

# 4.1 Process existence & identity
if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "Error: Gazebo process (PID: $GZ_PID) was TERMINATED by teardown!" >&2
    exit 1
fi

GZ_CMD=$(tr '\0' ' ' < "/proc/$GZ_PID/cmdline")
if [[ ! "$GZ_CMD" =~ gz[[:space:]]+sim ]]; then
    echo "Error: PID $GZ_PID is not a Gazebo process! Cmdline: $GZ_CMD" >&2
    exit 1
fi

GZ_STATE=$(awk '/State:/ {print $2, $3}' "/proc/$GZ_PID/status")
echo "  [VERIFIED] Gazebo process (PID: $GZ_PID) is ACTIVE: State = $GZ_STATE"
if [[ "$GZ_STATE" =~ Z ]]; then
    echo "Error: Gazebo process became a ZOMBIE!" >&2
    exit 1
fi

# 4.2 Instance uniqueness (ensure no other Gazebo instance is running)
OTHER_GZ=$(pgrep -f "gz sim" | grep -v "^$GZ_PID$" || true)
if [ -n "$OTHER_GZ" ]; then
    echo "Error: Multiple Gazebo instances detected on host! (Other PIDs: $OTHER_GZ)" >&2
    exit 1
fi
echo "  [VERIFIED] Single-instance isolation confirmed: only PID $GZ_PID is running."

# 4.3 Active simulation clock advancement (proves simulation is alive, not hung or spoofed)
echo "  Sampling simulation clock to verify continuous progression..."
source "/opt/ros/jazzy/setup.bash"
source "/home/capstone/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh" 2>/dev/null || true

T1_SEC=$(gz topic -e -t /clock -n 1 2>/dev/null | awk '/sec:/ {print $2; exit}')
sleep 1.0
T2_SEC=$(gz topic -e -t /clock -n 1 2>/dev/null | awk '/sec:/ {print $2; exit}')

echo "  Simulation Clock Sample 1: ${T1_SEC}s"
echo "  Simulation Clock Sample 2: ${T2_SEC}s (after 1.0s host sleep)"

if [ -z "$T1_SEC" ] || [ -z "$T2_SEC" ]; then
    echo "Error: Failed to sample /clock topic from Gazebo transport!" >&2
    exit 1
fi

if [ "$T2_SEC" -le "$T1_SEC" ]; then
    echo "Error: Simulation clock is frozen! T1=$T1_SEC, T2=$T2_SEC" >&2
    exit 1
fi
echo "  [VERIFIED] Simulation clock is actively advancing (T2 > T1: ${T2_SEC}s > ${T1_SEC}s)."

echo "----------------------------------------------------------------------"
echo "[4/4] Asserting PX4 SITL process health..."
echo "----------------------------------------------------------------------"

# 4.4 PX4 process existence & identity
if ! kill -0 "$PX4_PID" 2>/dev/null; then
    echo "Error: PX4 process (PID: $PX4_PID) was TERMINATED by teardown!" >&2
    exit 1
fi

PX4_CMD=$(tr '\0' ' ' < "/proc/$PX4_PID/cmdline")
if [[ ! "$PX4_CMD" =~ px4 ]]; then
    echo "Error: PID $PX4_PID is not a PX4 process! Cmdline: $PX4_CMD" >&2
    exit 1
fi

PX4_STATE=$(awk '/State:/ {print $2, $3}' "/proc/$PX4_PID/status")
echo "  [VERIFIED] PX4 process (PID: $PX4_PID) is ACTIVE: State = $PX4_STATE"
if [[ "$PX4_STATE" =~ Z ]]; then
    echo "Error: PX4 process became a ZOMBIE!" >&2
    exit 1
fi

echo "======================================================================"
echo "[PASS] REAL-PROCESS PRESERVATION FULLY DEMONSTRATED VIA LAUNCHER!"
echo "Production launcher strictly inhibited teardown (Exit 101)."
echo "Live Gazebo (PID: $GZ_PID) and PX4 (PID: $PX4_PID) were preserved alive."
echo "Instance isolation verified. Simulation clock confirmed actively progressing."
echo "======================================================================"

# Clean up verified test processes (strictly scoped by tracked PID and verified identity - Issue A Resolution)
echo "Cleaning up verified test processes and descendants..."
cleanup_all_tracked
echo "Done."
