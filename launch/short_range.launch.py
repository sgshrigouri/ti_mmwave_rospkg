from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ti_mmwave_rospkg',
            executable='ti_mmwave_rospkg',
            name='ti_mmwave',
            output='screen',
            parameters=[
                {'configFile': '/home/<your_username>/ros2_ws/src/ti_mmwave_rospkg/cfg/1642_short_range.cfg'},
                {'data_port': '/dev/ttyACM0'},
                {'config_port': '/dev/ttyACM1'},
                {'frame_id': 'ti_mmwave'}
            ]
        )
    ])