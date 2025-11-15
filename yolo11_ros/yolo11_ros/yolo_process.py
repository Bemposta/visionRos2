import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import json
from rclpy.time import Time as RclpyTime

class YoloProcess_ROS(Node):
    def __init__(self):
        super().__init__('yolo_process_ros')
        
        self.declare_parameter('classes', list(range(80)),)
        self.declare_parameter('show_frame', True)
        self.declare_parameter('verbose', True)
        self.classes = self.get_parameter('classes').get_parameter_value().integer_array_value
        self.show_frame = self.get_parameter('show_frame').get_parameter_value().bool_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.get_logger().info(f"Clases a detectar: {self.classes}")
        self.get_logger().info(f"show frame: {self.show_frame}")
        self.get_logger().info(f"verbose: {self.verbose}")

        self.subscription = self.create_subscription(CompressedImage, '/camera/image/compressed', self.listener_callback, 1)
        self.publisher_ = self.create_publisher(String, '/yolo/detctions', 1)
        self.subscription  # evitar advertencia de variable no usada
        self.publisher_

        self.get_logger().info('Cargando modelo YOLOv11n...')
        self.model = YOLO('yolo11n.pt')
        self.get_logger().info('Modelo cargado correctamente')

    def listener_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Diferencia en segundos
        if self.verbose:
            stamp = msg.header.stamp  # builtin_interfaces/Time
            msg_time = RclpyTime.from_msg(stamp)
            now_time = self.get_clock().now()
            delay = (now_time - msg_time).nanoseconds / 1e9
            self.get_logger().info(f"Mensaje publicado hace {delay:.3f} segundos")

        if frame is None:
            self.get_logger().warn("Imagen vacía o corrupta.")
            return

        results = self.model(frame, classes=self.classes, verbose=self.verbose)
        r = results[0]
        
        msg = String()
        msg.data = self.yolo_results_to_json(r, frame.shape[:2][::-1])
        self.publisher_.publish(msg)
        
        if self.show_frame:
            annotated_frame = r.plot()
            cv2.imshow("YOLOv11 detections", annotated_frame)
            cv2.waitKey(1)

    def yolo_results_to_json(self, results, sizeIm):
        detections = []
        for box in results.boxes:
            detection = {
                "class": int(box.cls.item()),  # clase detectada
                "confidence": float(box.conf.item()),  # nivel de confianza
                "xywh": box.xywh.tolist()[0]  # [x_center, y_center, width, height]
            }
            detections.append(detection)
        return json.dumps({"imgsz": sizeIm, "detections": detections}, skipkeys=True)

def main(args=None):
    rclpy.init(args=args)
    node = YoloProcess_ROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
