#include <micro_ros_arduino.h>
#include <trailbot_interfaces/srv/run_servo.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <Servo.h>
#include <string>

Servo servo1, servo2, servo3, servo4;


rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;
rcl_wait_set_t wait_set;

trailbot_interfaces__srv__RunServo_Response res;
trailbot_interfaces__srv__RunServo_Request req;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void operate_servo(Servo& servo, trailbot_interfaces__srv__RunServo_Response * res_in, const char * servo_name, const int32_t & time_us_1=1000, const int32_t & time_us_2=1500){
  servo.writeMicroseconds(time_us_1);
  delay(1200);
  servo.writeMicroseconds(time_us_2);
  delay(1200);
  res_in->success = true;
  // char msg_str[30] = "Operation succeeded: ";
  // strcat(msg_str, servo_name);
  res_in->message = micro_ros_string_utilities_set(res_in->message, "Operation succeeded");      
}

void service_callback(const void * req, void * res){
  trailbot_interfaces__srv__RunServo_Request * req_in = (trailbot_interfaces__srv__RunServo_Request *) req;
  trailbot_interfaces__srv__RunServo_Response * res_in = (trailbot_interfaces__srv__RunServo_Response *) res;

  int32_t servoId = req_in->servo;
  switch(servoId){
    case 1:
      operate_servo(servo1, res_in, "Servo1");  
      break;
    case 2:
      operate_servo(servo2, res_in, "Servo2");
      break;
    case 3:
      operate_servo(servo3, res_in, "Servo3");  
      break;
    case 4:
      operate_servo(servo4, res_in, "Servo4");
      break;
    default:
      res_in->success = false;      //light up LED
      res_in->message = micro_ros_string_utilities_set(res_in->message, "IncorrectServo");
      break;         
  }
  
}

void setup() {
  set_microros_transports();
  delay(1000); 

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "run_servo_service_node", "", &support));

  // create service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(trailbot_interfaces, srv, RunServo), "/runservo"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

  // define pwm out pins for servos
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
}


void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
