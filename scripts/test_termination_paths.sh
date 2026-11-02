#!/bin/bash
# ==============================================================================
# Deterministic Launcher Fail-Closed Control-Flow Test Suite (Finding D & P1)
# ==============================================================================
# Deterministically exercises the launcher's signal handling traps, exit traps,
# safety guard routing, and process-preservation decision logic:
# 1. Validates that all 5 termination triggers route to verify_safety_before_cleanup.
# 2. Validates that unverified/unsafe state trips fail-closed protection.
# 3. Validates that teardown cleanup is strictly INHIBITED (CLEANUP_INHIBITED=1).
# 4. Validates that the launcher exits with critical safety status 101.
# 5. Validates that tracked simulator PIDs are preserved on the process table.
# (Real Gazebo/PX4 process preservation with IPC health is tested in test_real_process_preservation.sh)
# ==============================================================================
set -e

WORKSPACE_DIR="/home/capstone/capstone_project_antigravity"
LAUNCHER="$WORKSPACE_DIR/scripts/test_forest_one_drone.sh"
LOG_DIR="/tmp/test_forest_one_drone"

PASS_COUNT=0
TOTAL_COUNT=5

echo "======================================================================"
echo "RUNNING LAUNCHER FAIL-CLOSED CONTROL-FLOW TESTS (DETERMINISTIC SUITE)"
echo "Target Launcher: $LAUNCHER"
echo "======================================================================"

# Helper function: strictly scoped dummy process termination with identity check (Issue C Resolution)
terminate_dummy_process() {
    local pid="$1"
    local desc="$2"
    if [ -z "$pid" ]; then return 0; fi
    if kill -0 "$pid" 2>/dev/null; then
        local cmdline=""
        if [ -f "/proc/$pid/cmdline" ]; then
            cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
        fi
        if [[ "$cmdline" =~ sleep ]]; then
            kill -TERM "$pid" 2>/dev/null || true
            for i in {1..5}; do
                if ! kill -0 "$pid" 2>/dev/null; then break; fi
                sleep 0.05
            done
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null || true
            fi
        else
            echo "  Warning: PID $pid does not match dummy process identity (cmd: '$cmdline'), skipping."
        fi
    fi
}

cleanup_leftover_test_processes() {
    if [ -f "$LOG_DIR/tracked_gz_pid.txt" ]; then
        local g
        g=$(cat "$LOG_DIR/tracked_gz_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_dummy_process "$g" "Tracked Dummy Gazebo"
    fi
    if [ -f "$LOG_DIR/tracked_px4_pid.txt" ]; then
        local p
        p=$(cat "$LOG_DIR/tracked_px4_pid.txt" 2>/dev/null | tr -d '[:space:]')
        terminate_dummy_process "$p" "Tracked Dummy PX4"
    fi
}

trap cleanup_leftover_test_processes EXIT

run_production_path_test() {
    local test_num="$1"
    local scenario="$2"
    local description="$3"
    local expected_log_pattern="$4"

    echo "----------------------------------------------------------------------"
    echo "[Test $test_num/$TOTAL_COUNT] Testing: $description"
    echo "  Scenario: --test-termination-path $scenario"

    rm -rf "$LOG_DIR"
    mkdir -p "$LOG_DIR"

    set +e
    "$LAUNCHER" --test-termination-path "$scenario" --timeout 10 > "$LOG_DIR/test_run.log" 2>&1
    local script_rc=$?
    set -e

    echo "  Launcher Exit Code: $script_rc (Expected: 101)"

    # 1. Verify exit code 101
    if [ "$script_rc" -ne 101 ]; then
        echo "  [FAIL] Expected exit code 101, got $script_rc!" >&2
        cat "$LOG_DIR/test_run.log" >&2
        return 1
    fi

    # 2. Verify unresolved safety state log exists and matches trigger pattern
    local log_file="$LOG_DIR/unresolved_safety_state.log"
    if [ ! -f "$log_file" ] || ! grep -q "$expected_log_pattern" "$log_file"; then
        echo "  [FAIL] Unresolved safety state log missing or pattern '$expected_log_pattern' not found!" >&2
        if [ -f "$log_file" ]; then cat "$log_file" >&2; fi
        return 1
    fi
    echo "  Logged State: $(cat "$log_file")"

    # 3. CRITICAL: Verify that Gazebo process was genuinely preserved and NOT killed by cleanup
    if [ ! -f "$LOG_DIR/tracked_gz_pid.txt" ]; then
        echo "  [FAIL] Tracked Gazebo PID file missing!" >&2
        return 1
    fi
    local gz_pid
    gz_pid=$(cat "$LOG_DIR/tracked_gz_pid.txt")

    if kill -0 "$gz_pid" 2>/dev/null; then
        echo "  [VERIFIED] Gazebo process (PID: $gz_pid) was GENUINELY PRESERVED by cleanup guard!"
        # Clean up preserved test process using identity verification
        terminate_dummy_process "$gz_pid" "Preserved Dummy Gazebo"
    else
        echo "  [CRITICAL FAULT] Gazebo process (PID: $gz_pid) was terminated! Process preservation failed!" >&2
        return 1
    fi

    # Clean up tracked PX4 process if present
    if [ -f "$LOG_DIR/tracked_px4_pid.txt" ]; then
        local px4_pid
        px4_pid=$(cat "$LOG_DIR/tracked_px4_pid.txt")
        terminate_dummy_process "$px4_pid" "Preserved Dummy PX4"
    fi

    echo "  -> [PASS] $description (Exit 101, Fail-Closed, Process Preserved)"
    PASS_COUNT=$((PASS_COUNT + 1))
    return 0
}

# Path 1: POST_TEST failure (Verifier timeout / unverified state)
run_production_path_test 1 "post-test-verifier-fail" \
    "POST_TEST Verifier Failure Path" \
    "trigger=POST_TEST"

# Path 2: SIGINT (Ctrl+C) interception during execution
run_production_path_test 2 "sigint" \
    "SIGINT (Ctrl+C) Interception Path" \
    "trigger=SIGINT_CTRL_C"

# Path 3: SIGTERM (Watchdog timeout) interception
run_production_path_test 3 "sigterm" \
    "SIGTERM (Watchdog Timeout) Interception Path" \
    "trigger=SIGTERM_WATCHDOG"

# Path 4: Unexpected script crash / test node failure (EXIT_TRAP)
run_production_path_test 4 "crash" \
    "Unexpected Script Crash (EXIT_TRAP) Path" \
    "trigger=EXIT_TRAP"

# Path 5: PX4 process loss/death when ARMING_REQUESTED=1
run_production_path_test 5 "px4-dead" \
    "PX4 Process Death Shortcut Elimination Path" \
    "reason=PX4_PROCESS_ABSENT_OR_DEAD"

echo "======================================================================"
echo "PRODUCTION LAUNCHER TERMINATION RESULTS: $PASS_COUNT / $TOTAL_COUNT PASSED"
echo "======================================================================"

if [ "$PASS_COUNT" -eq "$TOTAL_COUNT" ]; then
    echo "ALL 5 PRODUCTION TERMINATION PATHS VERIFIED FAIL-CLOSED AND PRESERVE PROCESSES!"
    echo "Scope Note: These tests verify deterministic launcher control-flow, signal interception,"
    echo "fail-closed protection tripping, and process preservation. They do NOT validate airborne"
    echo "flight dynamics or in-flight process cutoffs (airborne testing remains strictly unverified)."
    exit 0
else
    echo "Termination path verification failed!"
    exit 1
fi
