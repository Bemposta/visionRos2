from launch import LaunchDescription
from launch_ros.actions import Node

time_publisher = Node(
    package='time_test',
    executable='talker',
    name='talker'
)

time_listener = Node(
    package='time_test',
    executable='listener',
    name='listener'
)

def generate_launch_description():
    launchDescription = LaunchDescription([
        time_publisher,
        time_listener
    ])
    return launchDescription
