#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from aslan_msgs.msg import SDControl  # Import SDControl message

class WaypointFollowerNode:
    def __init__(self):
        rospy.init_node("waypoint_follower_node", anonymous=True)

        # Subscribers
        self.waypoints_sub = rospy.Subscriber("/lane_waypoints", MarkerArray, self.waypoints_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/sd_control", SDControl, queue_size=10)  # Control commands

        # Control parameters
        self.current_waypoint = None
        self.current_pose = None
        self.Kp_linear = 5.5  # Linear velocity gain
        self.Kp_angular = 3.0  # Angular velocity gain (reduced to avoid oscillations)
        self.waypoint_threshold = 1.2  # Threshold to check if the vehicle reached the waypoint
        self.max_linear_velocity = 40  # Maximum allowed linear velocity
        self.stopping = False  # Flag to indicate stopping
        self.prev_angular_velocity = 0.0  # To smooth out angular control

        # Distance tracking
        self.last_x = None
        self.last_y = None
        self.total_distance_traveled = 0.0  # Track total distance traveled
        self.stop_distance = 10.0  # Stop after 10 meters

        rospy.loginfo("Waypoint Follower Node Initialized")

    def waypoints_callback(self, msg):
        """ Callback for waypoints """
        if len(msg.markers) > 0:
            self.current_waypoint = msg.markers[0].pose.position  # Use the first waypoint
            rospy.loginfo("New waypoint: {}".format(self.current_waypoint))  
            self.stopping = False  # Reset stopping flag
        else:
            self.current_waypoint = None
            self.stopping = True  # Stop when no waypoints are available

    def odom_callback(self, msg):
        """ Callback for odometry """
        self.current_pose = msg.pose.pose

        # Track distance traveled
        if self.last_x is not None and self.last_y is not None:
            dx = self.current_pose.position.x - self.last_x
            dy = self.current_pose.position.y - self.last_y
            self.total_distance_traveled += np.sqrt(dx**2 + dy**2)

        self.last_x = self.current_pose.position.x
        self.last_y = self.current_pose.position.y

    def compute_control(self):
        """ Compute control commands to follow the waypoint or stop the vehicle """
        if self.total_distance_traveled >= self.stop_distance:
            rospy.loginfo("Stopping: Reached 10 meters!")
            self.stop_vehicle()
            return

        if self.stopping:
            self.stop_vehicle()
            return

        if self.current_waypoint is None or self.current_pose is None:
            rospy.logwarn("No waypoint or pose available!")
            self.stop_vehicle()
            return

        # Get current position and orientation
        x_current = self.current_pose.position.x
        y_current = self.current_pose.position.y
        orientation = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Get waypoint position
        x_target = self.current_waypoint.x
        y_target = self.current_waypoint.y

        # Compute errors
        dx = x_target - x_current
        dy = y_target - y_current
        distance = np.sqrt(dx**2 + dy**2)
        angle_to_target = np.arctan2(dy, dx)

        # Compute control commands
        linear_velocity = self.Kp_linear * distance
        angular_velocity = self.Kp_angular * (angle_to_target - yaw)

        # Apply deadzone to prevent small unnecessary corrections
        if abs(angular_velocity) < 2.0:
            angular_velocity = 0.0

        # Apply low-pass filter to smooth steering
        alpha = 0.7
        angular_velocity = alpha * self.prev_angular_velocity + (1 - alpha) * angular_velocity
        self.prev_angular_velocity = angular_velocity

        # Slow down before reaching the stopping point
        if self.total_distance_traveled >= self.stop_distance - 2:
            linear_velocity *= 0.5

        # Ensure max limits
        linear_velocity = min(self.max_linear_velocity, max(0, linear_velocity))
        angular_velocity = max(-100, min(100, angular_velocity))

        # Convert to SDControl message
        control_msg = SDControl()
        control_msg.torque = linear_velocity
        control_msg.steer = angular_velocity

        # Publish control
        self.cmd_vel_pub.publish(control_msg)
        rospy.loginfo("Command: torque={}, steer={}".format(control_msg.torque, control_msg.steer))

        # Stop at waypoint
        if distance < self.waypoint_threshold:
            rospy.loginfo("Reached waypoint!")
            self.current_waypoint = None
            self.stopping = True

    def stop_vehicle(self):
        """ Stop the vehicle by sending zero throttle and steering """
        control_msg = SDControl()
        control_msg.torque = 0  # Zero throttle
        control_msg.steer = 0  # Zero steering
        self.cmd_vel_pub.publish(control_msg)
        rospy.loginfo("Stopping vehicle: torque=0, steer=0")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.compute_control()
            rate.sleep()

if __name__ == "__main__":
    try:
        node = WaypointFollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
