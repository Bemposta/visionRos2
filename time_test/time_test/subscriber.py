import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as msgTime
from rclpy.time import Time as RclpyTime
from rclpy.qos import qos_profile_sensor_data

class TimeSubscriber(Node):
    def __init__(self):
        super().__init__('time_subscriber')
        self.subscription_time = self.create_subscription( msgTime, 'tiempo', self.listener_callback, qos_profile_sensor_data)
        self.subscription_time  # prevent unused variable warning

    def listener_callback(self, msg: msgTime):
        msg_time = RclpyTime.from_msg(msg)
        now_time = self.get_clock().now()
        delay = (now_time - msg_time).nanoseconds / 1e9
        self.get_logger().info(f"Delay: {delay:.4f} seg. {msg}")

def main(args=None):
    rclpy.init(args=args)
    time_subscriber = TimeSubscriber()
    rclpy.spin(time_subscriber)
    time_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
