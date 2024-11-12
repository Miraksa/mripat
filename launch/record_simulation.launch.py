from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='image_bridge_node',
            output='screen',
            arguments=['/camera']  # Pass the argument to the executable
        ),
        Node(
            package='mripat',
            executable='record_camera',
            name='recorder_node',
            output='screen'
        )
    ])