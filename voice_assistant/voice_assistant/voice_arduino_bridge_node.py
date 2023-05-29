import rclpy
from rclpy.node import Node

from trailbot_interfaces.srv import SnackWanted
from trailbot_interfaces.srv import RunServo


class VoiceArduinoBridge(Node):
    """
    This node is used to test the chatbot's logic independently of other nodes.  
    It plays the part of behaviour planner by creating a snack_wanted service.
    In the actual behaviour planner node, "snack_wanted" service should be created  
    and in its callback 'runservo' service should be called to operate the servo motors.

    1. Declare Parameters for available snacks, their quantity, snack2servo_map
    2. Create server for snack_wanted service
    3. Create client for runservo service 
    4. Create a launch file to launch both nodes

    """

    def __init__(self):
        super().__init__('voice_arduino_bridge_node')

        # Server for snack_wanted service
        self.srv = self.create_service(
            SnackWanted, 'snack_wanted', self.snack_wanted_callback)

        # TODO: client for runservo service
        # self.runservo_request

    # Send request to run servo service

    def send_request(self, servo):
        """
        Sends request to snack_wanted service 

        Args:
            snack (str): snack requested by user e.g. 'chips'

        Returns:
            response (bool, str): response of snack_wanted service 
                                    e.g. response.success = 'True', 
                                    response.message = 'Snack successfully dispensed'
        """

        self.runservo_request.servo = servo
        self.future = self.cli.call_async(self.runservo_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def snack_wanted_callback(self, request, response):

        snack = request.snack
        # TODO: check
        # if snack is available:
        #    send_request(servo) # calls run servo service
        #    if runservo service's response is successful,
        #       snack_wanted response.success = True
        #       snack_wanted response.message = 'Snack dispensed successfully'
        #    else:
        #       snack_wanted response.success = False
        #       snack_wanted response.message = 'Error: Servo ... did not operate.'
        # else:
        # response.success = False
        # response.message = 'not available'
        response.success = True
        response.message = f'{request.snack} dispensed successfully!'
        self.get_logger().info(f'Incoming request\nsnack: {request.snack}')
        self.get_logger().info(
            f'Response sent request\n {response.success}, {response.message}')

        return response


def main():
    rclpy.init()

    voice_arduino_bridge_node = VoiceArduinoBridge()

    rclpy.spin(voice_arduino_bridge_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
