from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Launch dobot_magician_control_system.launch.py
        ExecuteProcess(
            cmd=['ros2', 'launch', 'dobot_bringup', 'dobot_magician_control_system.launch.py'],
            output='screen',
        ),

        # 2. Wait for 5 seconds before the next launch
        TimerAction(
            period=5.0,
            actions=[
                # 3. Launch display.launch.py
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'dobot_description', 'display.launch.py', 'DOF:=4', 'tool:=suction_cup'],
                    output='screen',
                )
            ]
        ),

        # 4. Launch yolo node
        Node(
            package='gyeong_pkg',
            executable='yolo',
            name='yolo_node',
            output='screen',
        ),

        # 5. Launch dobot_mv node
        Node(
            package='gyeong_pkg',
            executable='dobot_mv',
            name='dobot_mv_node',
            output='screen',
        ),

        # 6. Launch robo_dk node
        Node(
            package='gyeong_pkg',
            executable='robo_dk',
            name='robo_dk_node',
            output='screen',
        ),
    ])
