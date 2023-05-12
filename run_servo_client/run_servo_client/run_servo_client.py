import sys

import rclpy
from rclpy.node import Node

from servo_interfaces.srv import RunServo


class RunServoClient(Node):

    def __init__(self):
        super().__init__('run_servo_client_node')
        self.cli = self.create_client(RunServo, 'runservo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunServo.Request()

    def send_request(self, servo):
        self.req.servo = servo
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

def main():
    rclpy.init() #ros2 run run_servo_client client 3

    run_servo_client_node = RunServoClient()

    # Run Servo and Dispense Snack
    servo_id = int(sys.argv[1]) #can be 1,2,3,4
    response = run_servo_client_node.send_request(servo_id)
    
    # Print Response
    run_servo_client_node.get_logger().info(f'Response of running servo: {servo_id} -> {response.success}, {response.message}')
    run_servo_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()