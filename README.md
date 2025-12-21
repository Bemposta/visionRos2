# visionRos2

Códigos para implementar visión artificial en ROS2

---

# Lanzadores

Estos lanzadores lanzan varios paquetes a la vez.

Lanzador del reconocedor __YOLO__  
`ros2 launch src/visionRos2/launch/yolo_launch.py`

Lanzador de detector de color __Rojo__  
`ros2 launch src/visionRos2/launch/colcor_detect_launch.py`

---

# Paquetes:

## webcam_cpub

Publica y visualiza las imágenes de una WebCam, o de un video.  
Publica las imágenes en RAW o Comprimidas.  
Este nodo __no utiliza cv_brige__

### lanzador

`ros2 launch webcam_cpub webcam_launch.py`

### nodo publisher

publisher: src/compressed_image_publisher.cpp  
publicador de imágenes desde WebCam o desde fichero en formato comprimido (jpg).

#### parametros publisher

  - **camera_id**: *0* -> Si camera_id es < 0, entonces usa el video como fuente de imagenes.
  - **topic**: *"/camera/image"* -> El topic del video comprimido es topic+"/compressed"
  - **fps**: *2.0* -> Tiene que ser un double. No puede ser integer.
  - **video**: *"/home/mixi/ros2_ws/Tokyo_640.mp4"*

### nodo subscriber

subscriber: src/compressed_image_subscriber.cpp  
Visualizador de imágenes con openCV en formato comprimido. No usa cv_brige

## yolo11_ros

Procesa imágenes con la Red Neuronal de Yolo.  
Esta diseñado para trabajar junto a __webcam_cpub__

### lanzador

`ros2 launch yolo_process yolo_launch.py`

### nodo yolo_process

yolo_process: yolo11_ros.yolo_process:main  
Infiere las imágenes y devuelve los resultados en un json.

#### parametros publisher

- **classes**: *list(range(80))* -> Lista de clases a detectar.
- **show_frame**: *True* -> True muestra la imagen procesada y etiquetada.
- **verbose**: *True* -> True muestra información de tiempo de procesamiento.

## color_detect

Busca color rojo en imágenes publicadas en un topic de WebCam.  
Esta diseñado para trabajar junto a __webcam_cpub__

### lanzador

`ros2 launch src/visionRos2/launch/colcor_detect_launch.py`

### nodo red

red: src/red.cpp  
Busca el color rojo en imágenes en un topic comprimido (jpg).

### lanzador

`ros2 run color_detect red`

#### parametros:

  - **topicCam**: *"/camera/image"* -> El topic del video comprimido es topic+"/compressed".
  - **topicDetect**: *"/detect/red"* -> El topic del json procesado.
  - **showProcess**: *True* -> muestra la imagen procesada y etiquetada.
  - **debug**: *False* -> muestra información de procesamiento.

## time_test

Paquete para testear el tiempo entre mensajes.  
Se publica solo un topic con el __time__ actual. El subscriptor calcula el tiempo entre transmision y recepcion.

### lanzador

`ros2 launch test_time time_lanch.py`

### nodo publisher

publisher: src/publisher.cpp  
publicador del tempo actual.

### nodo subscriber

subscriber: src/subscriber.cpp  
Subscriptor al tiempo, y mustra el delay en el mensaje.
