#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/px4_ros/ws_px4_bridge/install/local_setup.bash
source ~/px4_ros/ws_uav_apps/install/local_setup.bash

echo "[OK] ROS2 env sourced:"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  AMENT_PREFIX_PATH (tail)=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | tail -n 3)"
