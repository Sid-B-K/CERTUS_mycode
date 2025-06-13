#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneDetectionNode:
    def __init__(self):
        rospy.init_node("lane_detection_node", anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/sd_twizy/front_center_camera/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/lane_detection/image", Image, queue_size=1)

        rospy.loginfo("Lane Detection Node Initialized")

    def preprocessing(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gblur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gblur, 50, 150)
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        return edges

    def preprocessing(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # grass mask to remove both green and brown tones
        lower_grass1 = np.array([25, 30, 30])
        upper_grass1 = np.array([90, 255, 255])  # bright green/yellow grass

        lower_grass2 = np.array([10, 50, 50])
        upper_grass2 = np.array([25, 255, 255])  # more brownish/dry patches

        grass_mask1 = cv2.inRange(hsv, lower_grass1, upper_grass1)
        grass_mask2 = cv2.inRange(hsv, lower_grass2, upper_grass2)
        grass_mask = cv2.bitwise_or(grass_mask1, grass_mask2)

        not_grass_mask = cv2.bitwise_not(grass_mask)

        filtered_img = cv2.bitwise_and(img, img, mask=not_grass_mask)

        gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)

        gblur = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(gblur, 50, 150)

        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        return edges



    def regionOfInterest(self, img, polygon):
        mask = np.zeros_like(img, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(polygon, np.int32)], 255)
        return cv2.bitwise_and(img, mask)

    def warp(self, img, src, dst, size):
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, matrix, size)

    def unwarp(self, img, src, dst, size):
        matrix = cv2.getPerspectiveTransform(dst, src)
        return cv2.warpPerspective(img, matrix, size)

    def fitCurve(self, img):
        histogram = np.sum(img[img.shape[0]//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 50
        margin = 100
        minpix = 30
        window_height = img.shape[0] // nwindows

        y, x = img.nonzero()
        leftx_current = leftx_base
        rightx_current = rightx_base
        left_lane_inds, right_lane_inds = [], []

        for window in range(nwindows):
            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((y >= win_y_low) & (y < win_y_high) & (x >= win_xleft_low) & (x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((y >= win_y_low) & (y < win_y_high) & (x >= win_xright_low) & (x < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(x[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(x[good_right_inds]))

        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
            leftx = x[left_lane_inds]
            lefty = y[left_lane_inds]
            rightx = x[right_lane_inds]
            righty = y[right_lane_inds]
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
            return left_fit, right_fit
        except Exception as e:
            rospy.logwarn("Curve fitting failed: %s", str(e))
            return None, None

    def findPoints(self, shape, left_fit, right_fit):
        if left_fit is None or right_fit is None:
            return np.array([[]]), np.array([[]])
        ploty = np.linspace(0, shape[0] - 1, shape[0])
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        return pts_left, pts_right

    def fillCurves(self, shape, pts_left, pts_right):
        if pts_left.size == 0 or pts_right.size == 0:
            return np.zeros((shape[0], shape[1], 3), dtype='uint8')
        pts = np.hstack((pts_left, pts_right))
        img = np.zeros((shape[0], shape[1], 3), dtype='uint8')
        cv2.fillPoly(img, np.int_([pts]), (255, 0, 0))
        return img

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            processed_img = self.preprocessing(frame)
            height, width = processed_img.shape

            polygon = [(int(width*0.15), int(height*0.74)),
                       (int(width*0.45), int(height*0.45)),
                       (int(width*0.58), int(height*0.45)),
                       (int(0.95*width), int(0.74*height))]
            masked_img = self.regionOfInterest(processed_img, polygon)

            src = np.float32([[int(width*0.49), int(height*0.45)],
                              [int(width*0.58), int(height*0.45)],
                              [int(width*0.15), int(height*0.74)],
                              [int(0.95*width), int(height*0.74)]])
            dst = np.float32([[0, 0], [400, 0], [0, 960], [400, 960]])
            warped_size = (400, 960)
            warped_shape = (960, 400)

            warped = self.warp(masked_img, src, dst, warped_size)
            morph = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))

            left_fit, right_fit = self.fitCurve(morph)
            pts_left, pts_right = self.findPoints(warped_shape, left_fit, right_fit)

            if pts_left.size > 0 and pts_right.size > 0:
                fill_img = self.fillCurves(warped_shape, pts_left, pts_right)
                unwarped = self.unwarp(fill_img, src, dst, (width, height))
                output = cv2.addWeighted(frame, 1, unwarped, 1, 0)
            else:
                output = frame

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))
        except Exception as e:
            rospy.logerr("Processing Error: %s", str(e))

if __name__ == "__main__":
    try:
        LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
