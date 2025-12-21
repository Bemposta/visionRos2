from launch import LaunchDescription
from launch_ros.actions import Node

time_clock = Node(
    package='time_test',
    executable='clock',
    name='clock'
)

time_echo = Node(
    package='time_test',
    executable='echo',
    name='echo'
)

def generate_launch_description():
    launchDescription = LaunchDescription([
        time_clock,
        time_echo,
    ])
    return launchDescription
