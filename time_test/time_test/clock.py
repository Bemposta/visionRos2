import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as msgTime
from rclpy.time import Time as RclpyTime
from rclpy.qos import qos_profile_sensor_data

class TimePing(Node):
    def __init__(self):
        super().__init__('time_ping')
        self.publicher_ping = self.create_publisher(msgTime, 'ping', qos_profile_sensor_data)
        self.subscription_echo = self.create_subscription( msgTime, 'ping_echo', self.echo_callback, qos_profile_sensor_data)
        self.subscription_echo  # prevent unused variable warning
        self.temporizador = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = msgTime()
        msg = self.get_clock().now().to_msg()
        self.publicher_ping.publish(msg)
        #self.get_logger().info(f'Publishing: "{msg}"')

    def echo_callback(self, msg: msgTime):
        msg_time = RclpyTime.from_msg(msg)
        now_time = self.get_clock().now()
        delay = (now_time - msg_time).nanoseconds / 1e6
        self.get_logger().info(f"Delay: {delay:6.2f} mseg. {msg}")

def main(args=None):
    rclpy.init(args=args)
    time_ping = TimePing()
    rclpy.spin(time_ping)
    time_ping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
