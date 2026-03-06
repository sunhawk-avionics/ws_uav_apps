#!/usr/bin/env bash
# ============================================================
# ROS 2 multi-workspace environment setup (stable version)
# Usage:
#   source Tools/setup_env.sh
# ============================================================

# --- guard: must be sourced, not executed ---
(return 0 2>/dev/null) || {
  echo "[ERROR] Please source this script:"
  echo "        source Tools/setup_env.sh"
  exit 1
}

# ---- save caller shell option states (because this file is SOURCED) ----
__had_errexit=0
__had_nounset=0
[[ $- == *e* ]] && __had_errexit=1
[[ $- == *u* ]] && __had_nounset=1

# We want robust behavior even if caller has `set -u` (Makefile does).
# Disable nounset while sourcing ROS setup; keep errors handled manually.
set +u
set +e

# --- workspace root (edit here if needed) ---
WS_ROOT="$HOME/sunhawk_ros2_ws"
WS_PX4_BRIDGE="$WS_ROOT/ws_px4_bridge"
WS_UAV_APPS="$WS_ROOT/ws_uav_apps"

# --- sanity checks ---
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble"
  # restore options before returning
  (( __had_nounset )) && set -u || set +u
  (( __had_errexit )) && set -e || set +e
  return 1
fi

# Optional: predefine to avoid ROS setup issues under nounset anyway
: "${AMENT_TRACE_SETUP_FILES:=}"

# --- source order matters ---
source /opt/ros/humble/setup.bash
__rc=$?
if [ $__rc -ne 0 ]; then
  echo "[ERROR] Failed to source /opt/ros/humble/setup.bash (rc=$__rc)"
  (( __had_nounset )) && set -u || set +u
  (( __had_errexit )) && set -e || set +e
  return $__rc
fi

if [ -f "$WS_PX4_BRIDGE/install/setup.bash" ]; then
  source "$WS_PX4_BRIDGE/install/setup.bash" || {
    echo "[WARN] Failed to source px4_bridge install/setup.bash"
  }
else
  echo "[WARN] px4_bridge not built:"
  echo "       $WS_PX4_BRIDGE/install/setup.bash missing"
fi

if [ -f "$WS_UAV_APPS/install/setup.bash" ]; then
  source "$WS_UAV_APPS/install/setup.bash" || {
    echo "[WARN] Failed to source ws_uav_apps install/setup.bash"
  }
else
  echo "[WARN] ws_uav_apps not built:"
  echo "       $WS_UAV_APPS/install/setup.bash missing"
fi

# --- restore caller options exactly as before ---
(( __had_nounset )) && set -u || set +u
(( __had_errexit )) && set -e || set +e

# --- diagnostics ---
echo "[OK] ROS 2 environment sourced"
echo "  ROS_DISTRO = ${ROS_DISTRO:-<unset>}"
echo "  AMENT_PREFIX_PATH (order):"
echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | nl -ba
