> 基于 **ROS 2 Humble + PX4 ROS Bridge** 的无人机仿真与调试工具集
>
> 用于 Sunhawk SITL / HIL 环境下的控制、传感器与发动机模型调试

## 1. 项目概述

`ws_uav_apps` 是 Sunhawk 无人机项目的 ROS 2 应用层工作空间，包含以下功能：

| 节点 / 功能 | 源文件 | 说明 |
|-------------|--------|------|
| 传感器数据监听 | `sensor_combined_listener.cpp` | 监听 uORB → ROS 2 的 `sensor_combined` |
| 调试向量发布 | `debug_vect_advertiser_sim_time.cpp` | 发布 DebugVect，支持仿真时间 |
| 发动机转速模型 | `engine_rpm_model_node.cpp` + `engine_sim.hpp` | SITL / 联调用发动机 RPM 模型 |
| 键盘手动控制 | `keyboard_manual_control_node_sdl2.cpp` | SDL2 键盘输入 → Manual Control 模拟 |
| 仿真统一启动 | `launch/sim_all.py` | 一键启动所有仿真节点 |

依赖的上游工作空间：

- `ws_px4_bridge`（提供 `px4_msgs` / `px4_ros_com`）

## 2. 目录结构

```text
ws_uav_apps/
├── Makefile                              # colcon 构建入口（debug / release / clean）
├── README.md
├── src/
│   └── sunhawk_debug/                    # ROS 2 包
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   ├── engine_rpm_model.yaml     #   发动机模型参数
│       │   └── keyboard_manual_control.yaml  #   键盘控制参数映射
│       ├── launch/
│       │   └── sim_all.py                #   仿真统一启动文件
│       └── src/
│           ├── sensor_combined_listener.cpp
│           ├── debug_vect_advertiser_sim_time.cpp
│           ├── engine_rpm_model_node.cpp
│           ├── engine_sim.hpp            #   发动机仿真模型头文件
│           └── keyboard_manual_control_node_sdl2.cpp
└── Tools/
    ├── setup_env.sh                      # 多工作空间环境初始化
    └── astyle/                           # 代码风格工具
        ├── astylerc
        ├── check_code_style_all.sh
        ├── check_code_style.sh
        ├── files_to_check_code_style.sh
        ├── fix_code_style.sh
        └── pre-commit
```

## 3. 编译

本工作空间通过 Makefile 封装 colcon，**编译前自动 source 环境**。

```bash
cd ws_uav_apps

make debug          # Debug 构建 (-O0 -g3)，支持断点调试
make release        # Release 构建
make clean          # 清理 build/install/log
make debug-clean    # clean + debug
make release-clean  # clean + release
```

其他辅助 target：

```bash
make env            # 打印当前 ROS 2 环境层级
make format         # 运行 astyle 代码格式化
make list-bin       # 列出 sunhawk_debug 已安装的可执行文件
make help           # 查看所有可用 target
```

> ⚠️ 编译前需确保 `ws_px4_bridge` 已编译且 `install/setup.bash` 存在，
> 否则会因缺少 `px4_msgs` 依赖而失败。

## 4. 运行

### 4.1 初始化环境

```bash
source Tools/setup_env.sh
```

### 4.2 一键仿真启动

```bash
ros2 launch sunhawk_debug sim_all.py
```

### 4.3 单独运行节点

```bash
# 键盘手动控制
ros2 run sunhawk_debug keyboard_manual_control_node \
  --ros-args --params-file $(ros2 pkg prefix sunhawk_debug)/share/sunhawk_debug/config/keyboard_manual_control.yaml

# 传感器监听
ros2 run sunhawk_debug sensor_combined_listener

# 发动机转速模型
ros2 run sunhawk_debug engine_rpm_model_node \
  --ros-args --params-file $(ros2 pkg prefix sunhawk_debug)/share/sunhawk_debug/config/engine_rpm_model.yaml
```

### 4.4 启动 Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 4.5 监听话题

```bash
ros2 topic echo /fmu/out/sensor_combined
```

## 5. 开发指南

### 5.1 新增节点的标准流程

在 `src/sunhawk_debug/CMakeLists.txt` 中添加：

```cmake
# 1. 编译目标
add_executable(my_new_node src/my_new_node.cpp)

# 2. 声明依赖
ament_target_dependencies(my_new_node rclcpp px4_msgs)

# 3. 安装
install(TARGETS my_new_node DESTINATION lib/${PROJECT_NAME})
```

如有新依赖，同步更新 `package.xml`：

```xml
<depend>rclcpp</depend>
<depend>px4_msgs</depend>
```

### 5.2 代码风格

```bash
make format                     # 自动格式化
Tools/astyle/check_code_style_all.sh   # 仅检查
```

## 6. 调试提示

| 问题 | 排查方向 |
|------|---------|
| 时间不同步 | ROS2 + Gazebo 必须使用 `/clock` |
| 消息收不到 | 确认 `px4_msgs` 与 firmware 中 `.msg` 一致 |
| 节点无输出 | 检查 QoS 设置（PX4 默认使用 `BestEffort`） |
| 找不到 ROS 2 包 | 先 `source Tools/setup_env.sh`，再执行 ROS 2 命令 |
| Debug 断点不停 | 确认使用 `make debug` 编译（`-O0 -g3`） |