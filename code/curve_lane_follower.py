#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion

class WaypointGenerator:
    def __init__(self):
        rospy.init_node('waypoint_generator')
        
        # Parameters
        self.waypoint_spacing = rospy.get_param('~waypoint_spacing', 1.0)
        self.max_deviation = rospy.get_param('~max_deviation', 0.5)
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.path_pub = rospy.Publisher('/waypoint_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=1)
        
        # Subscriber
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.loginfo("Waypoint Generator Initialized")

    def goal_callback(self, msg):
        try:
            # Transform goal from map to baselink frame
            transform = self.tf_buffer.lookup_transform('base_link',
                                                      msg.header.frame_id,
                                                      rospy.Time(0),
                                                      rospy.Duration(1.0))
            goal_in_base = do_transform_pose(msg, transform)
            
            # Generate path
            path = self.generate_path(goal_in_base)
            
            # Publish
            self.path_pub.publish(path)
            self.publish_waypoint_markers(path)
            rospy.loginfo("Published path to (%.2f, %.2f) with %d waypoints",
                         goal_in_base.pose.position.x,
                         goal_in_base.pose.position.y,
                         len(path.poses))
                         
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error: %s", str(e))
        except Exception as e:
            rospy.logerr("Error: %s", str(e))

    def generate_path(self, goal):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "base_link"  # Changed to standard frame name
        
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        distance = math.sqrt(goal_x**2 + goal_y**2)
        num_waypoints = max(1, int(distance / self.waypoint_spacing))
        
        for i in range(1, num_waypoints + 1):
            ratio = float(i) / num_waypoints
            waypoint = PoseStamped()
            waypoint.header = path.header
            
            # Position
            waypoint.pose.position.x = ratio * goal_x
            waypoint.pose.position.y = ratio * goal_y
            
            # Add natural curvature
            if num_waypoints > 3:
                deviation = self.max_deviation * math.sin(ratio * math.pi)
                angle = math.atan2(goal_y, goal_x)
                perp_angle = angle + math.pi/2
                waypoint.pose.position.x += deviation * math.cos(perp_angle)
                waypoint.pose.position.y += deviation * math.sin(perp_angle)
            
            # Orientation
            if i < num_waypoints:
                next_ratio = (i + 0.5) / num_waypoints
                next_x = next_ratio * goal_x
                next_y = next_ratio * goal_y
                yaw = math.atan2(next_y - waypoint.pose.position.y,
                                next_x - waypoint.pose.position.x)
            else:
                yaw = math.atan2(goal_y - waypoint.pose.position.y,
                                goal_x - waypoint.pose.position.x)
            
            waypoint.pose.orientation = self.yaw_to_quaternion(yaw)
            path.poses.append(waypoint)
        
        # Add final goal
        final_pose = PoseStamped()
        final_pose.header = path.header
        final_pose.pose = goal.pose
        path.poses.append(final_pose)
        
        return path

    def publish_waypoint_markers(self, path):
        marker_array = MarkerArray()
        
        for i, pose in enumerate(path.poses):
            marker = Marker()
            marker.header = pose.header
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0 if i == len(path.poses)-1 else 0.0
            marker.color.g = 0.0 if i == len(path.poses)-1 else 1.0
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q

if __name__ == '__main__':
    try:
        node = WaypointGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
