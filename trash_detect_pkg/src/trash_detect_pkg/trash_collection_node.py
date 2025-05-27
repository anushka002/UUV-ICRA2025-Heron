#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from heron_msgs.msg import Drive
import tf.transformations
import math
import numpy as np
import os
import re

class TrashCollectionController:
    def __init__(self):
        rospy.init_node('trash_collection_node', anonymous=True)

        self.trash_log_path = os.path.expanduser("~/catkin_ws/trash_detections.csv")
        self.trajectory_log_path = os.path.expanduser("~/catkin_ws/boat_trajectory.csv")
        self.waypoint_log_path = os.path.expanduser("~/catkin_ws/waypoints.csv")
        self.results_log_path = os.path.expanduser("~/catkin_ws/results.txt")
        self.truth_csv = os.path.expanduser("~/catkin_ws/trash_ground_truth.csv")

        with open(self.trash_log_path, 'wb') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])

        self.traj_log = open(self.trajectory_log_path, 'wb')
        self.traj_writer = csv.writer(self.traj_log)
        self.traj_writer.writerow(['x', 'y'])

        self.current_x = None
        self.current_y = None
        self.prev_x = None
        self.robot_heading = 0.0
        self.camera_matrix = None
        self.image_width = 640
        self.image_height = 480

        # Improved state tracking for detours
        self.detour_active = False      # True when executing a detour
        self.detour_cooldown = False    # True when in cooldown period after detour
        self.mission_direction = 1
        self.turning = False            # Flag to detect when the robot is turning
        self.prev_heading = None        # Store previous heading to detect turns
        self.turn_threshold = 0.1       # Threshold for detecting turns (in radians)

        # Performance metrics tracking
        self.detour_count = 0
        self.trash_detected_count = 0
        self.detour_paths = []
        self.mission_paths = []
        self.current_path_segment = []
        self.total_path_length = 0
        self.waypoint_recovery_count = 0
        self.waypoint_attempts = 0
        self.max_deviation = 0.0
        self.path_deviations = []

        self.pub_cmd = rospy.Publisher('/cmd_drive', Drive, queue_size=10)

        self.waypoints = self.generate_waypoints()
        self.current_wp_index = 0
        self.pending_detour = None
        self.detour_resume_wp_index = None

        rospy.Subscriber('/trash/locations', String, self.trash_callback)
        rospy.Subscriber('/front_camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/pose_gt', Odometry, self.pose_callback)

        rospy.loginfo("Trash Collection Node running...")
        self.rate = rospy.Rate(10)
        self.navigate()

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.image_width = msg.width
        self.image_height = msg.height

    def pose_callback(self, msg):
        self.prev_x = self.current_x
        self.current_y_prev = self.current_y if self.current_y is not None else None
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Update current path segment for later analysis
        if self.current_x is not None and self.current_y is not None:
            self.current_path_segment.append((self.current_x, self.current_y))
            
            # Calculate path length if previous position exists
            if self.prev_x is not None and self.current_y_prev is not None:
                segment_length = math.hypot(self.current_x - self.prev_x, self.current_y - self.current_y_prev)
                self.total_path_length += segment_length

        if self.prev_x is not None:
            delta_x = self.current_x - self.prev_x
            if abs(delta_x) > 0.01:
                self.mission_direction = 1 if delta_x > 0 else -1

        self.traj_writer.writerow([self.current_x, self.current_y])

        # Get robot heading (yaw)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        # Store previous heading for turn detection
        self.prev_heading = self.robot_heading if self.robot_heading is not None else euler[2]
        self.robot_heading = euler[2]
        
        # Detect if robot is turning by comparing current and previous heading
        if self.prev_heading is not None:
            heading_change = abs(self.normalize_angle(self.robot_heading - self.prev_heading))
            if heading_change > self.turn_threshold:
                if not self.turning:
                    #rospy.loginfo("[TURNING] Robot is turning, trash detection paused")
		    pass
                self.turning = True
            else:
                if self.turning:
                    #rospy.loginfo("[TURNING] Robot has stopped turning, trash detection resumed")
		    pass
                self.turning = False

        # Calculate path deviation metrics
        if not self.detour_active and self.current_wp_index > 0 and self.current_wp_index < len(self.waypoints):
            # Get the two bounding waypoints of the current segment
            prev_wp = self.waypoints[self.current_wp_index - 1]
            next_wp = self.waypoints[self.current_wp_index]
            
            # Calculate shortest distance from current position to line segment between waypoints
            deviation = self.point_to_line_distance(
                (self.current_x, self.current_y), 
                (prev_wp[0], prev_wp[1]), 
                (next_wp[0], next_wp[1])
            )
            
            self.path_deviations.append(deviation)
            self.max_deviation = max(self.max_deviation, deviation)

    def point_to_line_distance(self, point, line_point1, line_point2):
        """Calculate the shortest distance from a point to a line segment"""
        x0, y0 = point
        x1, y1 = line_point1
        x2, y2 = line_point2
        
        # If line segment is actually a point
        if (x1 == x2 and y1 == y2):
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        # Calculate the linear equation parameters
        A = y2 - y1
        B = x1 - x2
        C = x2*y1 - x1*y2
        
        # Calculate shortest distance
        distance = abs(A*x0 + B*y0 + C) / math.sqrt(A**2 + B**2)
        return distance

    def generate_waypoints(self):
        waypoints = []
        waypoints += [(0, 0), (3, 0), (6, 0), (9, 0), (12, 0)]
        waypoints += [(13.5, 1.5), (12, 3), (9, 3), (6, 3), (3, 3), (0, 3)]
        waypoints += [(-1.5, 4.5), (0, 6), (3, 6), (6, 6), (9, 6), (12, 6)]
        waypoints += [(13.5, 7.5), (12, 9), (9, 9), (6, 9), (3, 9), (0, 9)]
        waypoints += [(-1.5, 10.5), (0, 12), (3, 12), (6, 12), (9, 12), (12, 12)]

        with open(self.waypoint_log_path, 'wb') as f:
            writer = csv.writer(f)
            for wp in waypoints:
                writer.writerow([wp[0], wp[1]])

        return waypoints

    def is_left_or_right_patch(self, img_x, img_y):
        # Updated detection area as specified:
        # Top 10% excluded, bottom 60% excluded, left 10% excluded, right 10% excluded, center 32% excluded
        top_cutoff = int(self.image_height * 0.15)
        bottom_cutoff = int(self.image_height * 0.35)  # Only detect in the top 40%
        left_cutoff = int(self.image_width * 0.20)
        right_cutoff = int(self.image_width * 0.80)
        center_left = int(self.image_width * 0.34)
        center_right = int(self.image_width * 0.66)  # Center 32% excluded

        if img_y < top_cutoff or img_y > bottom_cutoff:
            return None
        if img_x < left_cutoff or img_x > right_cutoff:
            return None
        if center_left <= img_x <= center_right:
            return None

        return "left" if img_x < self.image_width / 2 else "right"

    def trigger_detour(self, patch_side, start_x, end_x):
        # Calculate the midpoint between the current waypoint and next waypoint
        midpoint_x = (start_x + end_x) / 2.0
        
        # Get the base y-coordinate from the current waypoint
        base_y = self.waypoints[self.current_wp_index][1]
        
        # Determine current segment direction
        segment_direction = 1 if end_x > start_x else -1
        
        # For detours, apply the rule:
        # If x is incrementing (direction = 1): right is negative offset, left is positive offset
        # If x is decrementing (direction = -1): right is positive offset, left is negative offset
        if patch_side == "right":
            offset_sign = -1 if segment_direction > 0 else 1
        else:  # left patch
            offset_sign = 1 if segment_direction > 0 else -1
        
        # Apply the offset to create the detour point
        offset_y = base_y + (offset_sign * 0.38)
        
        rospy.loginfo("[DETOUR] Triggered detour to: (%.2f, %.2f) from segment (%.2f, %.2f) to (%.2f, %.2f)", 
                     midpoint_x, offset_y, start_x, self.waypoints[self.current_wp_index][1], 
                     end_x, self.waypoints[self.current_wp_index + 1][1])
        
        self.pending_detour = (midpoint_x, offset_y)
        self.detour_active = True
        self.detour_count += 1
        
        # Save current path segment to mission paths before starting detour
        if self.current_path_segment:
            self.mission_paths.append(list(self.current_path_segment))
            self.current_path_segment = [(self.current_x, self.current_y)]  # Start new segment for detour
        
        # Store the current waypoint index to resume from after the detour
        self.detour_resume_wp_index = self.current_wp_index
        return True

    def send_drive_command(self, goal_x, goal_y):
        goal_tolerance = 0.4
        slow_turn_speed = 0.12
        forward_speed = 0.35
        timeout_counter = 0
        max_timeout = 300
        cmd = Drive()

        while not rospy.is_shutdown():
            if self.current_x is None or self.current_y is None:
                self.rate.sleep()
                continue

            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            dist = math.hypot(dx, dy)

            if dist < goal_tolerance:
                rospy.loginfo("[REACHED] Reached waypoint at (%.2f, %.2f)", goal_x, goal_y)
                break

            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.robot_heading)

            if abs(angle_diff) > 0.5:
                base = 0.0
                turn = slow_turn_speed if angle_diff > 0 else -slow_turn_speed
            elif abs(angle_diff) > 0.2:
                base = forward_speed * 0.5
                turn = (angle_diff / abs(angle_diff)) * slow_turn_speed
            else:
                base = forward_speed
                turn = angle_diff * 0.3

            cmd.left = max(min(base - turn, 1.0), -1.0)
            cmd.right = max(min(base + turn, 1.0), -1.0)
            self.pub_cmd.publish(cmd)

            timeout_counter += 1
            if timeout_counter > max_timeout:
                rospy.logwarn("[CONTROL] Timeout reached for waypoint (%.2f, %.2f)", goal_x, goal_y)
                break

            self.rate.sleep()

        cmd.left = 0.0
        cmd.right = 0.0
        self.pub_cmd.publish(cmd)
        rospy.sleep(1.0)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def trash_callback(self, msg):
        # Ignore trash detections during active detours, cooldown periods, or when turning
        if self.detour_active:
            #rospy.loginfo("[INFO] Ignoring trash detection - robot in active detour mode")
            return
        if self.detour_cooldown:
            #rospy.loginfo("[INFO] Ignoring trash detection - robot in detour cooldown")
            return
        if self.turning:
            #rospy.loginfo("[INFO] Ignoring trash detection - robot is turning")
            return

        lines = msg.data.strip().split("\n")
        for line in lines:
            match = re.search(r'(?:.*?:)?\s*\(?([+-]?\d+\.?\d*),\s*([+-]?\d+\.?\d*)\)?', line)
            if not match:
                continue
            img_x = float(match.group(1))
            img_y = float(match.group(2))

            patch_side = self.is_left_or_right_patch(img_x, img_y)
            if patch_side:
                self.trash_detected_count += 1
                rospy.loginfo("[INFO] Trash detected at %s patch", patch_side)
                
                # Make sure we're not at the end of the waypoint list
                if self.current_wp_index < len(self.waypoints) - 1:
                    start_x = self.waypoints[self.current_wp_index][0]
                    end_x = self.waypoints[self.current_wp_index + 1][0]
                    
                    # Record the detection regardless of detour activation
                    with open(self.trash_log_path, 'ab') as f:
                        writer = csv.writer(f)
                        writer.writerow([self.current_x, self.current_y])
                    
                    # Try to trigger a detour - this is now responsive to the boat's current position
                    if self.trigger_detour(patch_side, start_x, end_x):
                        # If detour was triggered, break to process it immediately
                        break

    def navigate(self):
        try:
            while not rospy.is_shutdown():
                # Handle detours with priority
                if self.detour_active and self.pending_detour:
                    goal_x, goal_y = self.pending_detour
                    rospy.loginfo("[DETOUR] Executing detour to (%.2f, %.2f)", goal_x, goal_y)
                    self.send_drive_command(goal_x, goal_y)
                    
                    # After reaching the detour point, save current path segment to detour paths
                    if self.current_path_segment:
                        self.detour_paths.append(list(self.current_path_segment))
                        self.current_path_segment = [(self.current_x, self.current_y)]  # Start new segment
                    
                    # After reaching the detour point
                    self.detour_active = False
                    self.pending_detour = None
                    
                    # Enable cooldown to prevent immediate re-triggering of detours
                    self.detour_cooldown = True
                    
                    # FIX: Instead of going back to previous waypoint, continue in mission direction
                    # Determine which waypoint to go to next based on mission_direction
                    if self.mission_direction > 0:
                        # Continue to next waypoint if moving in positive x direction
                        rospy.loginfo("[DETOUR] Completed, continuing to next waypoint %d in forward direction", 
                            self.current_wp_index + 1)
                    else:
                        # Continue to next waypoint if moving in negative x direction
                        rospy.loginfo("[DETOUR] Completed, continuing to next waypoint %d in reverse direction", 
                            self.current_wp_index + 1)
                    
                    # After a detour, we continue with the next waypoint in the mission
                    continue

                # Check if we've completed all waypoints
                if self.current_wp_index >= len(self.waypoints):
                    rospy.loginfo("[MISSION] All waypoints completed!")
                    break

                # Process the next mission waypoint
                goal_x, goal_y = self.waypoints[self.current_wp_index]
                
                # Log the waypoint we're moving to
                if self.detour_cooldown:
                    rospy.loginfo("[MISSION] Returning to mission plan at waypoint %d: (%.2f, %.2f)", 
                                self.current_wp_index, goal_x, goal_y)
                    self.waypoint_attempts += 1
                else:
                    rospy.loginfo("[MISSION] Moving to waypoint %d: (%.2f, %.2f)", 
                                self.current_wp_index, goal_x, goal_y)
                    
                self.send_drive_command(goal_x, goal_y)
                
                # Track successful waypoint recovery after detour
                if self.detour_cooldown:
                    self.waypoint_recovery_count += 1
                
                # Increment waypoint after reaching it
                self.current_wp_index += 1
                
                # Disable cooldown after reaching the next mission waypoint
                if self.detour_cooldown:
                    rospy.loginfo("[DETOUR] Cooldown deactivated, resuming normal operation")
                    self.detour_cooldown = False
                    
                rospy.sleep(0.5)
            
            # Save the last path segment
            if self.current_path_segment:
                if self.detour_active:
                    self.detour_paths.append(list(self.current_path_segment))
                else:
                    self.mission_paths.append(list(self.current_path_segment))
            
            # Calculate performance metrics at the end of mission
            self.calculate_performance_metrics()
            
        except Exception as e:
            rospy.logerr("Error in navigation: %s", str(e))
            # Try to calculate metrics even if there was an error
            self.calculate_performance_metrics()

    def calculate_performance_metrics(self):
        rospy.loginfo("[METRICS] Calculating performance metrics...")
        
        try:
            # 1. Calculate total mission area (bounding box of waypoints)
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            area_width = max(wp_x) - min(wp_x)
            area_height = max(wp_y) - min(wp_y)
            total_area = area_width * area_height
            
            # 2. Calculate theoretical path length (sum of all waypoint segments)
            theoretical_path_length = 0
            for i in range(1, len(self.waypoints)):
                segment_length = math.hypot(
                    self.waypoints[i][0] - self.waypoints[i-1][0],
                    self.waypoints[i][1] - self.waypoints[i-1][1]
                )
                theoretical_path_length += segment_length
            
            # 3. Calculate Coverage Rate
            coverage_rate = (self.total_path_length / theoretical_path_length) * 100
            
            # 4. Trash Collection Metrics
            trash_collection_rate = 0
            total_trash_objects = 0
            
            # Load ground truth trash locations if file exists
            trash_truth_locations = []
            if os.path.exists(self.truth_csv):
                with open(self.truth_csv, 'r') as f:
                    reader = csv.reader(f)
                    next(reader)  # Skip header
                    for row in reader:
                        trash_truth_locations.append((float(row[0]), float(row[1])))
                total_trash_objects = len(trash_truth_locations)
                
                # Load detected trash locations
                trash_detected_locations = []
                with open(self.trash_log_path, 'r') as f:
                    reader = csv.reader(f)
                    next(reader)  # Skip header
                    for row in reader:
                        trash_detected_locations.append((float(row[0]), float(row[1])))
                
                # Count unique trash objects detected (within 0.5m of ground truth)
                detection_radius = 0.5
                detected_count = 0
                for truth_loc in trash_truth_locations:
                    for detect_loc in trash_detected_locations:
                        dist = math.hypot(truth_loc[0] - detect_loc[0], truth_loc[1] - detect_loc[1])
                        if dist < detection_radius:
                            detected_count += 1
                            break
                
                if total_trash_objects > 0:
                    trash_collection_rate = (detected_count / total_trash_objects) * 100
            
            # 5. Path Deviation Metrics
            avg_deviation = np.mean(self.path_deviations) if self.path_deviations else 0
            
            # 6. Waypoint Recovery Success
            recovery_success_rate = 0
            if self.waypoint_attempts > 0:
                recovery_success_rate = (self.waypoint_recovery_count / self.waypoint_attempts) * 100
            
            # 7. Inter-line spacing (average distance between parallel survey lines)
            inter_line_spacing = 3.0  # Based on waypoint pattern
            
            # 8. Relative deviation as percentage of inter-line spacing
            relative_deviation = (avg_deviation / inter_line_spacing) * 100 if inter_line_spacing > 0 else 0
            
            # Write results to file - REPLACE f-strings with .format() for Python 2 compatibility
            with open(self.results_log_path, 'w') as f:
                f.write("TRASH COLLECTION MISSION PERFORMANCE METRICS\n")
                f.write("=========================================\n\n")
                f.write("Coverage Rate: {:.2f}%\n".format(coverage_rate))
                f.write("Detour Activations: {}\n".format(self.detour_count))
                f.write("Trash Collection Rate: {:.2f}%\n".format(trash_collection_rate))
                f.write("Path Deviation Magnitude (avg): {:.3f} meters\n".format(avg_deviation))
                f.write("Path Deviation Magnitude (peak): {:.3f} meters\n".format(self.max_deviation))
                f.write("Path Deviation (% of inter-line spacing): {:.1f}%\n".format(relative_deviation))
                f.write("Waypoint Recovery Success: {:.2f}%\n\n".format(recovery_success_rate))
                f.write("Total trash objects in environment: {}\n".format(total_trash_objects))
                f.write("Total trash detections: {}\n".format(self.trash_detected_count))
                f.write("Total path length: {:.2f} meters\n".format(self.total_path_length))
                f.write("Theoretical path length: {:.2f} meters\n".format(theoretical_path_length))
            
            rospy.loginfo("[METRICS] Performance metrics saved to %s", self.results_log_path)
            rospy.loginfo("[METRICS] Coverage Rate: {:.2f}%".format(coverage_rate))
            rospy.loginfo("[METRICS] Detour Activations: {}".format(self.detour_count))
            rospy.loginfo("[METRICS] Trash Collection Rate: {:.2f}%".format(trash_collection_rate))
            
        except Exception as e:
            rospy.logerr("[METRICS] Error calculating metrics: %s", str(e))

if __name__ == '__main__':
    try:
        TrashCollectionController()
    except rospy.ROSInterruptException:
        pass
