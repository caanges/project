#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf_transformations
import time
import json
from math import atan2, sqrt, pi
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        #self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # TF buffer and listener to get robot pose
        self.msg_data = PoseWithCovarianceStamped()
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)

        # Parameters
        #self.waypoints = [2.0, 4.0]
        self.waypoints = []
        self.current_waypoint_index = 0
        self.goal_tolerance = 0.2  # meters
        self.obstacle_distance_threshold = 0.4  # meters

        # Robot pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.robot_x_prev = None
        self.robot_y_prev = None
        self.robot_yaw_prev = None
        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Load waypoints from file (change path accordingly)
        self.load_waypoints('/home/rosdev/a_star_test2/path_coordinates.json')

        self.get_logger().info('Waypoint navigator started.')

    
    def load_waypoints(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                self.waypoints = [(float(p['y']), float(p['x'])) for p in data]
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
    
    def get_robot_pose(self):
        # Try to get transform from map to base_link (adjust frames if your tf tree differs)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        try:
            if self.msg_data is None:
                self.get_logger().warn("msg_data is None")
                return False
            #now = rclpy.time.Time()

            self.robot_x = self.msg_data.pose.pose.position.x
            self.robot_y = self.msg_data.pose.pose.position.y
            orientation_q = self.msg_data.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.robot_yaw = yaw
            return True
        except Exception as e:
            self.get_logger().warn(f'Could not get robot pose from TF: {e}')
            return False

    def listener_callback(self, msg):
        self.msg_data = msg

    def control_loop(self):

        if not self.get_robot_pose():
            self.get_logger().warn("Robot pose not available yet.")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Stopping.')
            self.stop_robot()
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]

        #goal_x = self.waypoints[0]
        #goal_y = self.waypoints[1]

        # Calculate distance and angle to goal
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)

        # Calculate angle difference between robot heading and goal direction
        angle_diff = angle_to_goal - self.robot_yaw
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi

        # Check if reached waypoint
        if distance < self.goal_tolerance:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}: ({goal_x:.2f}, {goal_y:.2f})")
            self.current_waypoint_index += 1
            self.stop_robot()
            return

        # Create Twist message to move robot toward goal
        cmd = Twist()

        # Angular velocity: proportional control to reduce angle difference
        cmd.angular.z = 1 * angle_diff
        self.get_logger().info(f"{angle_diff}")
        self.get_logger().info(f"going to {goal_x}, {goal_y}")

        # Linear velocity: move forward if roughly facing goal
        if abs(angle_diff) < 0.1:  
            # Slow down when near goal
            max_speed = 0.2
            if distance < 0.5:
                speed = max_speed * (distance / 0.5)
            else:
                speed = max_speed
            cmd.linear.x = speed
        else:
            cmd.linear.x = 0.0  # stop forward movement to turn first

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()