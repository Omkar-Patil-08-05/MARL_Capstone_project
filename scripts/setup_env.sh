#!/usr/bin/env bash
# ==============================================================================
# setup_env.sh — Project-Local Environment Setup for MARL Drone Swarm SAR
#
# Configures Gazebo Sim resource paths to locate project-local victim models
# and custom simulation models without repeatedly appending duplicate paths.
#
# Usage:
#   source scripts/setup_env.sh
#   OR
#   . scripts/setup_env.sh
# ==============================================================================

# Determine project root directory dynamically
if [ -n "${BASH_SOURCE[0]}" ]; then
    SCRIPT_PATH="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION}" ]; then
    SCRIPT_PATH="${(%):-%x}"
else
    SCRIPT_PATH="$0"
fi

SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" 2>/dev/null && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." 2>/dev/null && pwd)"

# Helper to append a directory to a colon-separated environment variable without duplication
add_to_path_var() {
    local var_name="$1"
    local dir_to_add="$2"
    local current_val="${!var_name}"

    if [ ! -d "${dir_to_add}" ]; then
        return 0
    fi

    # Check if directory is already present (exact match within colon boundaries)
    case ":${current_val}:" in
        *":${dir_to_add}:"*)
            # Already present, do nothing
            ;;
        *)
            if [ -z "${current_val}" ]; then
                export "${var_name}=${dir_to_add}"
            else
                export "${var_name}=${current_val}:${dir_to_add}"
            fi
            ;;
    esac
}

# 1. Project-local victim models directory
VICTIM_MODELS_DIR="${PROJECT_ROOT}/assets_real/victims"
add_to_path_var GZ_SIM_RESOURCE_PATH "${VICTIM_MODELS_DIR}"
add_to_path_var SDF_PATH "${VICTIM_MODELS_DIR}"

# 2. Project-local models directory
PROJECT_MODELS_DIR="${PROJECT_ROOT}/models"
add_to_path_var GZ_SIM_RESOURCE_PATH "${PROJECT_MODELS_DIR}"
add_to_path_var SDF_PATH "${PROJECT_MODELS_DIR}"

# 3. PX4 simulation assets if present
PX4_GZ_MODELS="/home/capstone/PX4-Autopilot/Tools/simulation/gz/models"
PX4_GZ_WORLDS="/home/capstone/PX4-Autopilot/Tools/simulation/gz/worlds"
add_to_path_var GZ_SIM_RESOURCE_PATH "${PX4_GZ_MODELS}"
add_to_path_var GZ_SIM_RESOURCE_PATH "${PX4_GZ_WORLDS}"

# Informative status when sourced interactively or executed
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    echo "[setup_env] Environment configured successfully."
    echo "[setup_env] GZ_SIM_RESOURCE_PATH contains: ${VICTIM_MODELS_DIR}"
else
    echo "=================================================================="
    echo "This script should be sourced into your current shell session:"
    echo "    source scripts/setup_env.sh"
    echo "Current GZ_SIM_RESOURCE_PATH would be:"
    echo "    ${GZ_SIM_RESOURCE_PATH}"
    echo "=================================================================="
fi
