from launch import LaunchDescription
from launch_ros.actions import Node

camera_publisher = Node(
    package='webcam_cpub',
    executable='publisher',
    name='camera_publisher',
    output='screen',
    parameters=[
        {"camera_id": 0},               # Si camera_id es <0, entonces usa el video como fuente de imagenes.
        {"topic": "/camera/image"},     # El topic del video comprimido es topic+"/compressed"
        {"fps": 5.0},                   # Tiene que ser un double. No puede ser integer.
        {"video": "/home/mixi/ros2_ws/src/visionRos2/Tokyo_640.mp4"}
    ]
)

color_detector = Node(
    package='color_detect',
    executable='red',
    name='red',
    output='screen',
    parameters=[
        {"topicCam": "/camera/image"},     # El topic del video comprimido es topic+"/compressed"
        {"topicDetect": "/detect/red"},    # El topic del json procesado
        {"showProcess": False},             # True muestra la imagen procesada y etiquetada.
        {"debug": False}                   # True muestra informacion de procesamiento.
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
        #camera_publisher,
        color_detector,
        camera_viewer
    ])
    return launchDescription
