#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class LaneDetectionNode:
    def __init__(self):
        rospy.init_node("lane_detection_node", anonymous=True)

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = rospy.Subscriber("/sd_twizy/front_center_camera/image_raw", Image, self.image_callback)

        # Publishers
        self.image_pub = rospy.Publisher("/lane_detection/image", Image, queue_size=1)
        self.waypoints_pub = rospy.Publisher("/lane_waypoints", MarkerArray, queue_size=1)

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Perspective transformation matrix (example, replace with actual calibration data)
        self.H = np.array([[1, 0, -300],
                           [0, 1, -900],
                           [0, 0, 1]], dtype=np.float32)

        # Initialize variables
        self.current_waypoint = None
        self.waypoint_threshold = 0.5  # Threshold to check if the vehicle reached the waypoint
        self.last_middle_lane = None  # Track the last detected middle lane
        self.marker_id = 0  # Unique ID for markers

        rospy.loginfo("Lane Detection Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image for lane detection
            lane_image, middle_lane = self.detect_lanes(cv_image)

            # Publish processed lane image
            processed_msg = self.bridge.cv2_to_imgmsg(lane_image, "bgr8")
            self.image_pub.publish(processed_msg)

            # Publish waypoint markers if middle lane detected
            if middle_lane:
                self.publish_waypoints(middle_lane)
                self.last_middle_lane = middle_lane  # Update the last detected middle lane
            else:
                if self.last_middle_lane is not None:
                    # If no middle lane is detected but there was a previous detection, delete waypoints
                    self.delete_waypoints()
                    self.last_middle_lane = None  # Reset the last detected middle lane

        except CvBridgeError as e:
            rospy.logerr("Error processing image: {}".format(e))

    def detect_lanes(self, img):
        """ Detect left and right lane edges, then compute middle lane line """

        # Get image dimensions
        height, width = img.shape[:2]

        # Ignore the bottom 25% of the image
        roi_height = int(height * 0.75)
        roi_img = img[:roi_height, :]

        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Adaptive Canny Edge Detection
        median_val = np.median(blur)
        lower_thresh = int(max(0, 0.66 * median_val))
        upper_thresh = int(min(255, 1.33 * median_val))
        edges = cv2.Canny(blur, lower_thresh, upper_thresh)

        # Region of Interest (ROI)
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (int(width * 0.1), roi_height),  
            (int(width * 0.9), roi_height),  
            (int(width * 0.6), int(roi_height * 0.6)),  
            (int(width * 0.4), int(roi_height * 0.6))   
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)

        # Apply ROI mask
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detect lane lines using Hough Transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=60, maxLineGap=150)

        # Separate left and right lane lines
        left_points, right_points = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / float(x2 - x1) if x2 - x1 != 0 else 0

                if abs(slope) < 0.3:  # Ignore horizontal lines
                    continue

                if slope < 0:  # Left lane
                    left_points.extend([(x1, y1), (x2, y2)])
                else:  # Right lane
                    right_points.extend([(x1, y1), (x2, y2)])

        # Fit lanes using least squares
        left_lane = self.fit_lane_line(left_points, roi_img.shape)
        right_lane = self.fit_lane_line(right_points, roi_img.shape)

        # Compute the middle lane line
        middle_lane = None
        if left_lane and right_lane:
            middle_lane = [
                (left_lane[0] + right_lane[0]) // 2, (left_lane[1] + right_lane[1]) // 2,
                (left_lane[2] + right_lane[2]) // 2, (left_lane[3] + right_lane[3]) // 2
            ]

        # Draw the detected lane lines
        lane_image = np.zeros_like(img)

        if left_lane:
            cv2.line(lane_image, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (255, 0, 0), 5)  
        if right_lane:
            cv2.line(lane_image, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0, 255, 0), 5)  
        if middle_lane:
            cv2.line(lane_image, (middle_lane[0], middle_lane[1]), (middle_lane[2], middle_lane[3]), (0, 0, 255), 5)  

        output = cv2.addWeighted(img, 0.8, lane_image, 1, 0)
        return output, middle_lane

    def fit_lane_line(self, points, img_shape):
        """ Fit a single lane line using least squares regression """
        if len(points) < 2:
            return None

        points = np.array(points)
        x_coords, y_coords = points[:, 0], points[:, 1]

        slope, intercept = np.polyfit(x_coords, y_coords, 1)

        y1 = img_shape[0]
        y2 = int(img_shape[0] * 0.6)

        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return (x1, y1, x2, y2)

    def publish_waypoints(self, middle_lane):
        """ Publish waypoints along the middle lane line and continuously update the waypoint """
        marker_array = MarkerArray()
        
        # Check if current waypoint exists, if not create a new one
        if self.current_waypoint is None:
            self.current_waypoint = self.create_waypoint(middle_lane)
        
        # Create a marker for the current waypoint
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lane_waypoints"
        marker.id = self.marker_id  # Assign a unique ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set waypoint position
        x_proj, y_proj = self.current_waypoint
        marker.pose.position.x = x_proj / 100.0  # Convert from pixels to meters
        marker.pose.position.y = y_proj / 100.0
        marker.pose.position.z = 0.0  # Keep it at ground level

        # Set marker appearance
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Add the marker to the array
        marker_array.markers.append(marker)

        # Publish the marker array
        self.waypoints_pub.publish(marker_array)

        # Increment the marker ID for the next waypoint
        self.marker_id += 1

        # Once the vehicle reaches the current waypoint, create a new one
        if self.has_reached_waypoint(x_proj, y_proj):
            self.current_waypoint = self.create_waypoint(middle_lane)

    def delete_waypoints(self):
        """ Delete all waypoints by publishing a DELETE action for each marker """
        marker_array = MarkerArray()
        for marker_id in range(self.marker_id):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "lane_waypoints"
            marker.id = marker_id
            marker.action = Marker.DELETE  # Delete the marker
            marker_array.markers.append(marker)

        # Publish the marker array to delete the markers
        self.waypoints_pub.publish(marker_array)
        rospy.loginfo("Deleted all waypoints")

        # Reset the marker ID counter
        self.marker_id = 0

    def create_waypoint(self, middle_lane):
        """ Create a new waypoint based on the middle lane """
        x1, y1, x2, y2 = middle_lane
        start_point = np.array([[x1, y1]], dtype=np.float32)

        # Perform perspective projection
        start_point_projected = cv2.perspectiveTransform(start_point.reshape(1, -1, 2), self.H)
        x_proj, y_proj = start_point_projected[0][0]

        return x_proj, y_proj

    def has_reached_waypoint(self, x_proj, y_proj):
        """ Check if the vehicle has reached the waypoint """
        try:
            # Wait for the transform to become available
            self.tf_listener.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            vehicle_x, vehicle_y = trans[0], trans[1]

            # Calculate the distance to the waypoint
            distance = np.sqrt((vehicle_x - x_proj / 100.0) ** 2 + (vehicle_y - y_proj / 100.0) ** 2)
            return distance < self.waypoint_threshold

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF error: {}".format(e))
            rospy.logwarn("Falling back to default position (0, 0)")
            return False  # Assume the vehicle has not reached the waypoint

if __name__ == "__main__":
    try:
        lane_detection_node = LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Lane Detection Node interrupted")
