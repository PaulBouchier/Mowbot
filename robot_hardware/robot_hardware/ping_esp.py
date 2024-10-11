import rclpy
from rclpy.node import Node

from time import time

from std_msgs.msg import Int32, Float32

class PingEsp(Node):
    def __init__(self):
        super().__init__('ping_esp')

        self.pong_sub = self.create_subscription(Int32, 'pong_esp', self.pong_callback, 10)
        self.ping_pub = self.create_publisher(Int32, 'ping_esp', 10)
        self.ping_timer = self.create_timer(5, self.ping_timer_callback)  # ping esp32 every 5 sec
        self.ping_rqst_msg = Int32()
        self.ping_rqst_msg.data = 0

    def ping_timer_callback(self):
        self.get_logger().info("ping_esp publishing ping request")
        self.ping_pub.publish(self.ping_rqst_msg)
        self.last_ping_time = self.get_clock().now()

    def pong_callback(self, esp_time):
        ping_response_time = self.get_clock().now() - self.last_ping_time
        self.get_logger().info("ping_esp received pong after {} with esp timestamp {}".format(ping_response_time, esp_time.data))


def main(args=None):
    rclpy.init(args=args)

    ping_esp = PingEsp()

    rclpy.spin(ping_esp)

    ping_esp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
