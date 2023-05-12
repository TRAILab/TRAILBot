from trailbot_interfaces.srv import SnackWanted

import rclpy
from rclpy.node import Node


class SnackWantedService(Node):

    def __init__(self):
        super().__init__('snack_wanted_service_node')
        self.srv = self.create_service(SnackWanted, 'snack_wanted', self.snack_wanted_callback)

    def snack_wanted_callback(self, request, response):

        response.success = True
        response.message = f'{request.snack} dispensed successfully!'
        self.get_logger().info(f'Incoming request\nsnack: {request.snack}')
        self.get_logger().info(f'Response sent request\n {response.success}, {response.message}')

        return response


def main():
    rclpy.init()

    snack_wanted_service_node = SnackWantedService()

    rclpy.spin(snack_wanted_service_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
