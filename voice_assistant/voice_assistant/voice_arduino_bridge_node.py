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

<<<<<<< Updated upstream
    1. Declare Parameters for available snacks, their quantity, snack2servo_map
    2. Publish available snacks + their quantity on a topic "available_snacks" at 1 Hz
    3. In voice assistant node: subscribe to the topic "available_snacks" and set a member variable
    4. Create server for snack_wanted service
    5. Create client for runservo service 
=======
    1. Declare Parameters for available snacks, their quantity, snack2servo_map - DONE
    2. Create server for snack_wanted service - DONE
    3. Create client for runservo service - DONE
    4. Create a launch file to launch both nodes - DONE, but needs checking (an error)

    additional:
    Figure out a way to make the snack parameters global/shareable,
    and be able to easily update them when the vending machine is refilled.
>>>>>>> Stashed changes

    """

    def __init__(self):
        super().__init__('voice_arduino_bridge_node')

        # Declare params
        self.declare_parameter(
            'snack_options', ['chips', 'chocolates', 'candies', 'nuts'])
        self.declare_parameter('available_snacks_quantity', [5, 5, 5, 5])
        self.declare_parameter('snack2servos_map', [1, 2, 3, 4])

        # Server for snack_wanted service
        self.srv = self.create_service(
            SnackWanted, 'snack_wanted', self.snack_wanted_callback)
        
        # Set up snack_wanted client
        self.cli = self.create_client(RunServo, 'runservo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('runservo service not available, waiting again...')
        self.runservo_request = RunServo.Request()

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

        # how do I know which snack they're looking for? What function/node is that coming from?
        # is that request.snack?
        # take i as the index of snack
        i = 1 # fow now
        if self.get_parameter('available_snacks_quantity').value[i] != 0:

        # TODO: check
        # if snack is available:
        #    send_request(servo) # calls run servo service
        #    if runservo service's response is successful,
            response = self.send_request(RunServo)
            self.get_logger().info(
                f'Response of RunServo service request: {response.success}, {response.message}')

            if response.success:
                response.success = True
                response.message = f'{request.snack} dispensed successfully!'
        #       Update snack quantity parameter
        #       will this line actually update the parameter?
                self.get_parameter('available_snacks_quantity').value[i] -= 1        # change the i later
            else:
                response.success = False
                response.message = f'Sorry, {request.snack} is not currently available.'
            
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
