from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('sunhawk_debug')

    engine_param = os.path.join(
        pkg_share, 'config', 'engine_rpm_model.yaml'
    )

    keyboard_param = os.path.join(
        pkg_share, 'config', 'keyboard_manual_control.yaml'
    )

    # 1. micro-ros-agent（外部进程）
    micro_ros_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '--port', '8888', '-v'],
        output='log'
    )

#     clock_bridge = ExecuteProcess(
#         cmd=[
#                 'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
#                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'
#         ],
#         output='log'
#     )

    # 2. 发动机仿真节点（带 yaml）
    engine = Node(
        package='sunhawk_debug',
        executable='engine_rpm_model_node',
        name='engine_rpm_model',
        output='log',
        parameters=[engine_param],
    )

    # 3. 键盘控制
    keyboard = Node(
        package='sunhawk_debug',
        executable='keyboard_manual_control_node_sdl2',
        name='keyboard_control',
        output='screen',
        parameters=[keyboard_param],
    )


    return LaunchDescription([
        micro_ros_agent,
        # clock_bridge,
        engine,
        keyboard,
    ])
