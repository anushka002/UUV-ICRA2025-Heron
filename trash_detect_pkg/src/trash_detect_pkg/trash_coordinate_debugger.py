#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
import tf.transformations
import math
import numpy as np

class TrashCoordinateDebugger:
    def __init__(self):
        rospy.init_node('trash_coordinate_debugger', anonymous=True)

        self.current_x = 0.0
        self.current_y = 0.0
        self.robot_heading = 0.0
        self.camera_matrix = None
        self.dist_coeffs = None

        self.image_width = 640  # default
        self.image_height = 480  # default

        self.detour_mode = False
        self.mission_direction = 1  # 1 for increasing x, -1 for decreasing x

        rospy.Subscriber('/trash/locations', String, self.trash_callback)
        rospy.Subscriber('/front_camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/pose_gt', Odometry, self.pose_callback)

        rospy.loginfo("Trash Coordinate Debugger is running...")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        self.image_width = msg.width
        self.image_height = msg.height
        rospy.loginfo("[DEBUG] Camera info received.")

    def pose_callback(self, msg):
        new_x = msg.pose.pose.position.x
        self.mission_direction = 1 if new_x > self.current_x else -1
        self.current_x = new_x
        self.current_y = msg.pose.pose.position.y

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot_heading = euler[2]

    def is_left_or_right_patch(self, img_x, img_y):
        top_threshold = int(self.image_height * 0.4)
        if img_y > top_threshold:
            return None

        if img_x < self.image_width / 2:
            return "left"
        else:
            return "right"

    def trigger_detour(self, patch_side, start_x, end_x):
        midpoint_x = (start_x + end_x) / 2.0
        offset_y = 0.375 if patch_side == "left" else -0.375
        if self.mission_direction < 0:
            offset_y *= -1

        rospy.loginfo("[DETOUR] Triggered detour to: (%.2f, %.2f)", midpoint_x, offset_y)
        # Insert command here to move the boat to (midpoint_x, offset_y)

        self.detour_mode = True
        # After reaching detour target, detour_mode must be reset externally

    def transform_image_to_world(self, img_x, img_y):
        if self.camera_matrix is None:
            return None

        try:
            camera_height = 0.3
            camera_forward = 0.4
            camera_pitch = 0.35

            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]

            x_normalized = (img_x - cx) / fx
            y_normalized = (img_y - cy) / fy
            ray_dir = np.array([x_normalized, y_normalized, 1.0])
            ray_dir = ray_dir / np.linalg.norm(ray_dir)

            camera_rotation = np.array([
                [np.cos(camera_pitch), 0, np.sin(camera_pitch)],
                [0, 1, 0],
                [-np.sin(camera_pitch), 0, np.cos(camera_pitch)]
            ])
            camera_translation = np.array([camera_forward, 0, camera_height])

            ray_dir_base = np.dot(camera_rotation, ray_dir)
            camera_pos_base = camera_translation

            if ray_dir_base[2] >= -0.001:
                return None

            t = -camera_pos_base[2] / ray_dir_base[2]
            intersection_base = camera_pos_base + t * ray_dir_base

            c = np.cos(self.robot_heading)
            s = np.sin(self.robot_heading)
            rotation_z = np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ])

            intersection_world = np.dot(rotation_z, intersection_base[:3]) + np.array([self.current_x, self.current_y, 0])

            return intersection_world

        except Exception as e:
            rospy.logwarn("[DEBUG] Error in coordinate transformation: %s", str(e))
            return None

    def trash_callback(self, msg):
        if self.detour_mode:
            rospy.loginfo("[DEBUG] Currently in detour mode. Ignoring detection.")
            return

        lines = msg.data.strip().split("\n")
        for line in lines:
            try:
                if ':' not in line:
                    continue

                _, coord_part = line.split(":", 1)
                coord_part = coord_part.strip().replace("(", "").replace(")", "")
                coords = [float(c.strip()) for c in coord_part.split(",") if c.strip()]
                if len(coords) != 2:
                    continue

                img_x, img_y = coords

                rospy.loginfo("[DEBUG] Trash detected at image coords: (%.2f, %.2f)", img_x, img_y)
                rospy.loginfo("[DEBUG] Image dimensions: width=%d, height=%d", self.image_width, self.image_height)

                patch_side = self.is_left_or_right_patch(img_x, img_y)

                if patch_side:
                    rospy.loginfo("[INFO] Trash detected at %s patch", patch_side)
                    current_wp_x = round(self.current_x / 3.0) * 3
                    next_wp_x = current_wp_x + 3 * self.mission_direction
                    self.trigger_detour(patch_side, current_wp_x, next_wp_x)
                else:
                    rospy.loginfo("[DEBUG] Trash not in top-left/right patch. img_y = %.2f, threshold = %.2f",
                                  img_y, self.image_height * 0.4)

            except Exception as e:
                rospy.logwarn("[DEBUG] Failed to process trash message: %s", str(e))

if __name__ == '__main__':
    try:
        TrashCoordinateDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

