import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as msgTime
from rclpy.qos import qos_profile_sensor_data

class TimeEcho(Node):
    def __init__(self):
        super().__init__('time_echo')
        self.subscription_ping = self.create_subscription( msgTime, 'ping', self.ping_callback, qos_profile_sensor_data)
        self.subscription_ping  # prevent unused variable warning
        self.publicher_echo = self.create_publisher(msgTime, 'ping_echo', qos_profile_sensor_data)

    def ping_callback(self, msg: msgTime):
        self.publicher_echo.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    time_echo = TimeEcho()
    rclpy.spin(time_echo)
    time_echo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
