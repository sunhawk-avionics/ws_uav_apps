# ws_uav_apps

> 基于 **ROS 2 Humble + PX4 ROS Bridge** 的无人机仿真与调试工具集
>
> 用于 **PX4 SITL / HIL 环境下的控制、传感器与发动机转速模型调试等**

---

## 1. 项目背景与用途

`ws_uav_apps` 是一个用于 PX4 飞控调试的 ROS 2 工程工作区间，主要功能包括：

* 🔹 监听 PX4 uORB → ROS2 的传感器数据（如 `sensor_combined`）
* 🔹 发布调试向量（DebugVect），用于分析控制与仿真行为
* 🔹 提供 **发动机转速模型节点**（Engine RPM Model），用于 SITL / 联调
* 🔹 提供 **键盘手动控制节点**，模拟 RC / Manual Control 输入
* 🔹 统一管理仿真启动流程（launch）

该工程通常与以下工作区配合使用：

* `ws_px4_bridge`（px4_msgs / px4_ros_com）
* Gazebo / ROS-GZ Bridge
* Micro XRCE Agent

---

## 2. 目录结构说明

```text
px4_ros/ws_uav_apps/
├── build/                     # colcon 构建产物
├── install/                   # 安装空间
├── log/                       # 运行日志
├── src/
│   └── sunhawk_debug/
│       ├── config/
│       │   └── engine_rpm_model.yaml   # 发动机转速模型参数
│       ├── launch/
│       │   └── sim_all.py               # 仿真统一启动文件
│       ├── src/
│       │   ├── sensor_combined_listener.cpp
│       │   ├── debug_vect_advertiser_sim_time.cpp
│       │   ├── engine_rpm_model_node.cpp
│       │   ├── keyboard_manual_control_node_sdl2.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── Tools/                     # 工程工具（代码风格、pre-commit）
├── Makefile
├── .gitignore
└── README.md
```

---

## 3. 构建 ROS 2 节点

### 3.1 创建包（示例）

```bash
cd ~/px4_ros/ws_uav_apps/src
ros2 pkg create sunhawk_debug \
  --build-type ament_cmake \
  --dependencies rclcpp px4_msgs
```

---

### 3.2 修改 `CMakeLists.txt`（必须）

示例：添加一个传感器监听节点

```cmake
add_executable(sensor_combined_listener src/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)

install(TARGETS sensor_combined_listener
  DESTINATION lib/${PROJECT_NAME}
)
```

> 每新增一个节点，都遵循 **add_executable → ament_target_dependencies → install** 的三步模式。

---

### 3.3 修改 `package.xml`（可选）

```xml
<depend>rclcpp</depend>
<depend>px4_msgs</depend>

<!-- 若涉及消息运行时 -->
<exec_depend>rosidl_default_runtime</exec_depend>
```

---

## 4. 编译整个工作区

```bash
# 1. 环境变量
cd ~/px4_ros/ws_uav_apps
bash Tools/setup_env.sh

# 2. 当前工程
colcon build
source install/local_setup.bash
```

---

## 5. 仿真与运行流程

### 5.1 启动通信与仿真时间

```bash
# XRCE Agent
MicroXRCEAgent udp4 -p 8888

# Gazebo / ROS 时间同步
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```
---

### 5.2 运行节点示例

#### 键盘手动控制节点

```bash
ros2 run sunhawk_debug keyboard_manual_control_node \
  --ros-args -p output_topic:=/fmu/in/manual_control_setpoint
```

#### 监听 PX4 uORB → ROS2 消息

```bash
ros2 topic echo /fmu/out/sensor_combined
```

---

## 6. 常见调试建议

* **时间问题**：PX4 + Gazebo 必须使用 `/clock`
* **消息不通**：确认 `px4_msgs` 与 firmware 中 `.msg` 一致
* **节点无输出**：检查 QoS（PX4 默认使用 BestEffort）