# visionRos2
Códigos para implementar visión artificial en ROS2

# Lanzador
`ros2 launch ./src/visionRos2/launch/vision_system_launch.py`

# Paquetes:

## webcam_cpub

### publisher
<p>publisher: src/compressed_image_publisher.cpp
<p>publicador de imagenes desde WebCam o desde fichero en formato comprimido (jpg). No usa cv_brige.

### subscriber
<p>subscriber: src/compressed_image_subscriber.cpp
<p>Visualizador de imagenes con openCV en formato comprimido. No usa cv_brige

### lanzador
`ros2 launch webcam_cpub webcam_launch.py`
- parametros publisher
  - **camera_id**: *0* -> Si camera_id es <0, entonces usa el video como fuente de imagenes.
  - **topic**: *"/camera/image"* -> El topic del video comprimido es topic+"/compressed"
  - **fps**: *2.0* -> Tiene que ser un double. No puede ser integer.
  - **video**: *"/home/mixi/ros2_ws/Tokyo_640.mp4"*

## yolo11_ros

### yolo_process
<p>yolo_process: yolo11_ros.yolo_process:main
<p>Infiere las imagenes y devuelve los resultados en un json.

### lanzador
`ros2 launch yolo_process yolo_launch.py`
- **classes**: *list(range(80))* -> Lista de clases a detectar.
- **show_frame**: *True* -> True muestra la imagen procesada y etiquetada.
- **verbose**: *True* -> True muestra informacion de tiempo de procesamiento.
