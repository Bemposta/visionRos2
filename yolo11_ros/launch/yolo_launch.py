from launch import LaunchDescription
from launch_ros.actions import Node

yolo_detector = Node(
    package='yolo11_ros',
    executable='yolo_process',
    name='yolo_detector',
    output='screen',
    emulate_tty=True,  # Para ver logs coloreados
    parameters=[
        {"classes": [0, 1, 2, 3, 4, 5, 6]}, #Lista de clases a detectar.
        {"show_frame": True},               #True muestra la imagen procesada y etiquetada.
        {"verbose": False},                 #True muestra informacion de tiempo de procesamiento.
    ]
)

def generate_launch_description():
    launchDescription = LaunchDescription([
        yolo_detector
    ])
    return launchDescription
