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

set -e

# --- workspace root (edit here if needed) ---
WS_ROOT="$HOME/px4_ros"
WS_PX4_BRIDGE="$WS_ROOT/ws_px4_bridge"
WS_UAV_APPS="$WS_ROOT/ws_uav_apps"

# --- sanity checks ---
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble"
  return 1
fi

# --- source order matters ---
source /opt/ros/humble/setup.bash

if [ -f "$WS_PX4_BRIDGE/install/setup.bash" ]; then
  source "$WS_PX4_BRIDGE/install/setup.bash"
else
  echo "[WARN] px4_bridge not built:"
  echo "       $WS_PX4_BRIDGE/install/setup.bash missing"
fi

if [ -f "$WS_UAV_APPS/install/setup.bash" ]; then
  source "$WS_UAV_APPS/install/setup.bash"
else
  echo "[WARN] ws_uav_apps not built:"
  echo "       $WS_UAV_APPS/install/setup.bash missing"
fi

# --- diagnostics ---
echo "[OK] ROS 2 environment sourced"
echo "  ROS_DISTRO = $ROS_DISTRO"
echo "  AMENT_PREFIX_PATH (order):"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | nl -ba
