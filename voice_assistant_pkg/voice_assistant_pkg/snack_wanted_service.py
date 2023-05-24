import rclpy
from rclpy.node import Node

from trailbot_interfaces.srv import SnackWanted


class SnackWantedService(Node):
    """
    This node is used to test the chatbot's logic independently of other nodes.  
    It plays the part of behaviour planner by creating a snack_wanted service.
    In the actual behaviour planner node, "snack_wanted" service should be created  
    and in its callback 'runservo' service should be called to operate the servo motors.
    """

    def __init__(self):
        super().__init__('snack_wanted_service_node')
        self.srv = self.create_service(
            SnackWanted, 'snack_wanted', self.snack_wanted_callback)

    def snack_wanted_callback(self, request, response):

        response.success = True
        response.message = f'{request.snack} dispensed successfully!'
        self.get_logger().info(f'Incoming request\nsnack: {request.snack}')
        self.get_logger().info(
            f'Response sent request\n {response.success}, {response.message}')

        return response


def main():
    rclpy.init()

    snack_wanted_service_node = SnackWantedService()

    rclpy.spin(snack_wanted_service_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
