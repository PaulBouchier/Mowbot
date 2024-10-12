import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class ClearOdomClient(Node):
    def __init__(self):
        super().__init__('clear_odom')
        self.cli = self.create_client(SetBool, 'clear_odom')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear_odom service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    clear_odom_client = ClearOdomClient()
    future = clear_odom_client.send_request()
    rclpy.spin_until_future_complete(clear_odom_client, future)
    response = future.result()
    if response.success is True:
        clear_odom_client.get_logger().info("Sent clear_odom command")
    else:
        clear_odom_client.get_logger().info("clear_odom command was not accepted")

    clear_odom_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
