#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class TrashColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/front_camera/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/trash/image_raw', Image, queue_size=1)
        self.location_pub = rospy.Publisher('/trash/locations', String, queue_size=10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Preprocessing for robustness
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # More tolerant color ranges
        colors = {
            "red": [
                ([0, 70, 50], [10, 255, 255]), 
                ([160, 70, 50], [179, 255, 255])
            ],
            "green": [([35, 50, 50], [85, 255, 255])],
            "yellow": [([15, 70, 50], [35, 255, 255])]
        }

        detections = []

        for color, ranges in colors.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                mask |= cv2.inRange(hsv, lower_np, upper_np)

            # Morphological operations to reduce noise
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx = x + w // 2
                    cy = y + h // 2
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, color, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    detections.append("{}: ({}, {})".format(color, cx, cy))

        # Publish processed image
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_msg)

        if detections:
            self.location_pub.publish(String(data="\n".join(detections)))

if __name__ == '__main__':
    try:
        TrashColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

