from launch import LaunchDescription
from launch_ros.actions import Node

camera_publisher = Node(
    package='webcam_cpub',
    executable='publisher',
    name='camera_publisher',
    output='screen',
    parameters=[
        {"camera_id": 0},              #Si camera_id es <0, entonces usa el video como fuente de imagenes.
        {"topic": "/camera/image"},     #El topic del video comprimido es topic+"/compressed"
        {"fps": 2.0},                   #Tiene que ser un double. No puede ser integer.
        {"video": "/home/mixi/ros2_ws/src/visionRos2/Tokyo_640.mp4"}
    ]
)

camera_viewer = Node(
    package='webcam_cpub',
    executable='subscriber',
    name='camera_viewer',
    output='screen'
)

def generate_launch_description():
    launchDescription = LaunchDescription([
        camera_publisher,
        #camera_viewer
    ])
    return launchDescription
