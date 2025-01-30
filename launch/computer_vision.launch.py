from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='ros_gz_image',
        #     executable='image_bridge',
        #     name='image_bridge_node',
        #     output='screen',
        #     arguments=['/camera']  # Pass the argument to the executable
        # ),
        Node(
            package='mripat',
            executable='test_camera',
            name='camera',
            output='screen'
        ),
        Node(
            package='mripat',
            executable='computer_vision',
            name='computer_vision',
            output='screen'
        ),
        Node(
            package='mripat',
            executable='object_detection',
            name='object_detection',
            output='screen'
        ),
    ])