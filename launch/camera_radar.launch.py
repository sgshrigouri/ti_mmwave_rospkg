
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
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'camera'}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_to_camera',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'ti_mmwave', 'camera']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/<your_username>/ros2_ws/src/ti_mmwave_rospkg/rviz/ti_mmwave.rviz']
        )
    ])