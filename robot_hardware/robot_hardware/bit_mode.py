import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class BitModeClient(Node):
    def __init__(self):
        super().__init__('bit_mode')
        self.cli = self.create_client(SetBool, 'bit_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('bit_mode service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    bit_mode_client = BitModeClient()
    future = bit_mode_client.send_request()
    rclpy.spin_until_future_complete(bit_mode_client, future)
    response = future.result()
    if response.success is True:
        bit_mode_client.get_logger().info("Sent bit_mode command")
    else:
        bit_mode_client.get_logger().info("bit_mode command was not accepted")

    bit_mode_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
