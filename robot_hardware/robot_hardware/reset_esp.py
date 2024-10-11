import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class ResetEspClient(Node):
    def __init__(self):
        super().__init__('reset_esp')
        self.cli = self.create_client(SetBool, 'reset_esp')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset_esp service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    reset_esp_client = ResetEspClient()

    reset_esp_client = ResetEspClient()
    future = reset_esp_client.send_request()
    rclpy.spin_until_future_complete(reset_esp_client, future)
    response = future.result()
    reset_esp_client.get_logger().info("Sent reset_esp command")

    reset_esp_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
