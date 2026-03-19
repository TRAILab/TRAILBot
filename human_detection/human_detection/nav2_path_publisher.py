import os
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from omegaconf import OmegaConf
from tf_transformations import quaternion_from_euler
import math
from scipy.spatial.transform import Rotation as R

class Nav2PathCommander(Node):
    def __init__(self, config):
        super().__init__('nav2_path_commander')
        self.navigator = BasicNavigator()
        self.navigator.lifecycleStartup()  # Cartographer-compatible

        self.path_file = config.get("global_path_file",
            "/home/trailbot/Documents/data_process_Kitti/results/04/planed_path/global_path_raw.json")

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.last_mtime = None
        self.sent = False
        self.husky_state = None

        self.get_logger().info(f"Watching for path at: {self.path_file}")

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/husky_pose',
            self.pose_callback,
            10
        )
    
    def pose_callback(self, msg: PoseStamped):
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_z = msg.pose.position.z

        quat = msg.pose.orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        current_roll = rotation.as_euler('xyz')[0]
        current_pitch = rotation.as_euler('xyz')[1]
        current_yaw = rotation.as_euler('xyz')[2]
        self.husky_state = [current_x, current_y, current_z, current_roll, current_pitch, current_yaw] 
        # self.get_logger().info(f"Current pose: x={current_x}, y={current_y}, z={current_z}, yaw={current_yaw}")


    def timer_callback(self):
        if self.husky_state is None:
            self.get_logger().warn("Waiting for current Husky pose before processing path...")
            return
        if not os.path.exists(self.path_file):
            return

        mtime = os.path.getmtime(self.path_file)
        if self.last_mtime == mtime:
            return

        self.last_mtime = mtime
        try:
            with open(self.path_file, 'r') as f:
                data = json.load(f)
            self.get_logger().info(f"____Processed____: {self.path_file}")

            if data.get('trigger', False):
                path_points = data.get('path', [])
                self.send_path_to_nav2(path_points)

                # Reset trigger
                data['trigger'] = False
                self.get_logger().info(f"reset the trigger to ::::::::: {data['trigger']}")
                with open(self.path_file, 'w') as f:
                    json.dump(data, f)
                self.get_logger().info(f"Processed path with {len(path_points)} points.")
        except Exception as e:
            self.get_logger().error(f"Failed to process path file: {e}")


    def send_path_to_nav2(self, path_points):
        if not path_points:
            self.get_logger().warn("Path is empty, skipping...")
            return
        # Cancel current goal if one is running
        # if self.navigator.is_task_active():
        #     self.get_logger().info("Cancelling current navigation task...")
        #     self.navigator.cancelTask()
        #     # Give Nav2 a short moment to cancel cleanly
        #     time.sleep(0.5)

        try:
            self.get_logger().info("Cancelling any active navigation task (if any)...")
            self.navigator.cancelTask()
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"Cancel task failed or not needed: {e}")
        poses = []
        if len(path_points) ==1:
            self.get_logger().warn("Path has less than 2 points")
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = path_points[0][0]
            goal.pose.position.y = path_points[0][1]
            goal.pose.position.z = path_points[0][2] if len(path_points[0]) > 2 else 0.0

            if self.husky_state is not None:
                current_x = self.husky_state[0]
                current_y = self.husky_state[1]

                dx = path_points[0][0] - current_x
                dy = path_points[0][1] - current_y
                yaw = math.atan2(dy, dx)
            else:
                self.get_logger().warn("No current pose available, using fallback orientation.")
                yaw = 0.0  # Default orientation if no current pose is available    

            q = quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            self.get_logger().info("Sending single goal with orientation based on current pose...")
            self.navigator.goToPose(goal)
        else:
            for i, point in enumerate(path_points):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2] if len(point) > 2 else 0.0

                if i < len(path_points) - 1:
                    dx = path_points[i+1][0] - point[0]
                    dy = path_points[i+1][1] - point[1]
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = self.husky_state[-1] #if self.husky_state is not None else 0.0  # Use current yaw if available

                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            
                poses.append(pose)

            self.get_logger().info(f"Sending {len(poses)} poses to Nav2 via goThroughPoses...")
            self.navigator.goThroughPoses(poses[1:])

        # while not self.navigator.isTaskComplete():
        #     rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation result: {result}")

def main(args=None):
    config_path = '/home/trailbot/trail_ws/src/TRAILBot/object_nav/object_nav/config/scenegraph.yaml'
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init(args=args)
    node = Nav2PathCommander(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()