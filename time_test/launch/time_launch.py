from launch import LaunchDescription
from launch_ros.actions import Node

time_publisher = Node(
    package='pubsub',
    executable='talker',
    name='talker'
)

time_listener = Node(
    package='pubsub',
    executable='listener',
    name='listener'
)

def generate_launch_description():
    launchDescription = LaunchDescription([
        time_publisher,
        time_listener
    ])
    return launchDescription
