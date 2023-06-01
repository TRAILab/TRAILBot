import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from trailbot_interfaces.msg import SnacksInventory
from trailbot_interfaces.srv import RunServo, SnackWanted


class VoiceArduinoBridge(Node):
    """
    This node manages snack inventory and forwards snack_wanted service request from 
    voice_assistant node to runsero service in arduino node.

    """

    def __init__(self):
        super().__init__('voice_arduino_bridge_node')

        # Declare params
        self.declare_parameter(
            'snack_options', ['chips', 'chocolate', 'candies', 'nuts'])
        self.declare_parameter('snack_quantity', [5, 5, 5, 5])
        self.declare_parameter('snack2servos_map', [1, 2, 3, 4])

        # Server for snack_wanted service
        self.srv = self.create_service(
            SnackWanted, 'snack_wanted', self.snack_wanted_callback)

        # Set up runservo client
        self.cli = self.create_client(RunServo, 'runservo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('runservo service not available, waiting again...')
        self.runservo_request = RunServo.Request()

        # Set up available_snacks publisher
        self.publisher = self.create_publisher(
            SnacksInventory, 'snack_inventory', 10)
        timer_period = 1  # publishes every second
        # Create the timer
        self.timer = self.create_timer(
            timer_period, self.publisher_timer_callback)

    def send_request(self, servo):
        """
        Sends request to runservo service 

        Args:
            servo (int): servo to operate e.g. 1,2,3,4

        Returns:
            response (bool, str): response of runservo service 
                                    e.g. response.success = 'True', 
                                    response.message = 'Operation Succeeded: Servo<ID>'
        """

        self.runservo_request.servo = servo
        self.future = self.cli.call_async(self.runservo_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def snack_wanted_callback(self, request, response):

        snack = request.snack

        snack_options = self.get_parameter('snack_options').value
        snack_quantities = self.get_parameter(
            'snack_quantity').value

        if snack not in snack_options:
            self.get_logger.error(f'Unknown Snack Requested: {snack}!')
            response.success = False
            response.message = f'{request.snack} is not available.'
            return response

        ind = snack_options.index(snack)
        if snack_quantities[ind] == 0:
            response.success = False
            response.message = f'Sorry, we have run out of {request.snack}.'
            return response
        else:
            servos = self.get_parameter('snack2servos_map').value
            servo_id = servos[ind]
            runservo_service_response = self.send_request(servo_id)
            assert runservo_service_response.success, \
                f"ERROR: Servo ID {servo_id} is not between 1 and 4. Check snack2servos_map!"

            response.success = True
            response.message = f'{request.snack} dispensed successfully!'

            # Update snack quantity parameter
            snack_quantities[ind] -= 1
            param_quantity = Parameter(
                'snack_quantity', Parameter.Type.INTEGER_ARRAY, snack_quantities)
            self.set_parameters([param_quantity])

        return response

    def publisher_timer_callback(self):
        """
        Callback function - publishes snack inventory.
        Gets called every second.
        """
        snack_options = self.get_parameter(
            'snack_options').value
        snack_quantities = self.get_parameter(
            'snack_quantity').value

        msg = SnacksInventory()
        msg.snacks = snack_options
        msg.quantity = snack_quantities
        self.publisher.publish(msg)


def main():
    rclpy.init()

    voice_arduino_bridge_node = VoiceArduinoBridge()

    rclpy.spin(voice_arduino_bridge_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
