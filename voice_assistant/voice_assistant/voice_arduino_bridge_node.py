import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from trailbot_interfaces.srv import SnackWanted
from trailbot_interfaces.srv import RunServo


class VoiceArduinoBridge(Node):
    """
    This node is used to test the chatbot's logic independently of other nodes.  
    It plays the part of behaviour planner by creating a snack_wanted service.
    In the actual behaviour planner node, "snack_wanted" service should be created  
    and in its callback 'runservo' service should be called to operate the servo motors.

    1. Declare Parameters for available snacks, their quantity, snack2servo_map - DONE
    2. Create server for snack_wanted service - DONE
    3. Create client for runservo service - DONE
    4. Create a launch file to launch both nodes - DONE, but needs checking

    additional:
    Figure out a way to make the snack parameters global/shareable,
    and be able to easily update them when the vending machine is refilled.

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

        # Set up available_snacks publisher
        self.publisher_ = self.create_publisher(String, 'available_snacks', 10)
        timer_period = 1  # publishes every second
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize a counter variable
        self.i = 0

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

        snack_options = self.get_parameter('snack_options').value
        snack_quantities = self.get_parameter('available_snack_quantity').value
        ind = snack_options.index(snack)
        if snack_quantities[ind] == 0:
            response.success = False
            response.message = f'Sorry, {request.snack} is not currently available.'
        else:     # (if snack is available)
        #    send_request(servo) - calls run servo service
        #    if runservo service's response is successful:
    
            servos = self.get_parameter('snack2servos_map').value
            servo_id = servos[ind]
            runservo_service_response = self.send_request(servo_id)
            assert runservo_service_response.success, f"ERROR: Servo ID {servo_id} is not between 1 and 4."

            self.get_logger().info(
                f'Response of RunServo service request: {response.success}, {response.message}')

            response.success = True
            response.message = f'{request.snack} dispensed successfully!'
    #       Update snack quantity parameter
            snack_quantities[ind] -= 1
            param_quantity = Parameter('available_snacks_quantity', Parameter.Type.LIST, snack_quantities)
            self.set_parameters([param_quantity])
            
        self.get_logger().info(f'Incoming request\nsnack: {request.snack}')
        self.get_logger().info(
            f'Response sent request\n {response.success}, {response.message}')

        return response


    def available_snacks_callback(self):
        """
        Callback function - gives the current quantities of snacks.
        Gets called every second.
        """
        snack_quantities = self.get_parameter('available_snack_quantity').value

        msg = String()
        msg.data = 'The snack quantities are: %d' % snack_quantities
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # Increment the counter by 1    
        self.i += 1

def main():
    rclpy.init()

    voice_arduino_bridge_node = VoiceArduinoBridge()

    rclpy.spin(voice_arduino_bridge_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
