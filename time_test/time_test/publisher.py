import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as msgTime
from rclpy.time import Time as RclpyTime
from rclpy.qos import qos_profile_sensor_data

class TimePublisher(Node):
    def __init__(self):
        super().__init__('timer_publisher')
        self.publicher_time = self.create_publisher(msgTime, 'tiempo', qos_profile_sensor_data)
        self.temporizador = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = msgTime()
        msg = self.get_clock().now().to_msg()
        self.publicher_time.publish(msg)
        #self.get_logger().info(f'Publishing: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    time_publisher = TimePublisher()
    rclpy.spin(time_publisher)
    time_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
