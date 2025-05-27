#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from heron_msgs.msg import Drive
import math
import os

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_controller', anonymous=True)

        self.pub = rospy.Publisher('/cmd_drive', Drive, queue_size=10)
        rospy.Subscriber('/pose_gt', Odometry, self.odom_callback)

        self.current_x = None
        self.current_y = None
        self.yaw = 0.0

        self.waypoints = self.generate_waypoints()
        rospy.loginfo("Generated %d waypoints", len(self.waypoints))

        # Save waypoints to CSV
        with open('/home/anushka/catkin_ws/waypoints.csv', 'w') as f:
            for x, y in self.waypoints:
                f.write("{:.2f},{:.2f}\n".format(x, y))

        self.rate = rospy.Rate(10)
        self.navigate()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

        # Log real-time trajectory
        with open('/home/anushka/catkin_ws/boat_trajectory.csv', 'a') as f:
            f.write("{:.2f},{:.2f}\n".format(self.current_x, self.current_y))

    def generate_waypoints(self):
        waypoints = []
        direction = 1
        y = 0.0
        max_x = 10.0
        step_x = 2.0
        step_y = 3.0
        curve_offset = 1.5
        num_rows = 5

        for row in range(num_rows):
            if direction == 1:
                x_points = [x for x in range(0, int(max_x)+1, int(step_x))]
            else:
                x_points = [x for x in range(int(max_x), -1, -int(step_x))]

            for x in x_points:
                waypoints.append((x, y))

            if row < num_rows - 1:
                curve_x = max_x + curve_offset if direction == 1 else -curve_offset
                waypoints.append((curve_x, y + step_y / 2))
                y += step_y
                waypoints.append((x_points[-1], y))
                direction *= -1

        return waypoints

    def navigate(self):
        while self.current_x is None or self.current_y is None:
            rospy.loginfo("Waiting for pose_gt data...")
            rospy.sleep(1)

        visited = set()

        for i, (target_x, target_y) in enumerate(self.waypoints):
            waypoint_key = (round(target_x, 1), round(target_y, 1))
            if waypoint_key in visited:
                continue
            rospy.loginfo("Waypoint %d: Navigating to (%.2f, %.2f)", i, target_x, target_y)
            self.go_to_waypoint(target_x, target_y)
            rospy.loginfo("Waypoint %d: Reached (%.2f, %.2f)", i, target_x, target_y)
            visited.add(waypoint_key)
            rospy.sleep(0.5)

    def go_to_waypoint(self, target_x, target_y):
        goal_tolerance = 0.4
        slow_turn_speed = 0.12
        forward_speed = 0.35

        while not rospy.is_shutdown():
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)

            if distance < goal_tolerance:
                self.send_cmd(0.0, 0.0)
                break

            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.yaw)

            if abs(angle_diff) > 0.5:
                angular_vel = slow_turn_speed if angle_diff > 0 else -slow_turn_speed
                self.send_cmd(0.0, angular_vel)
            elif abs(angle_diff) > 0.2:
                linear = forward_speed * 0.5
                angular = (angle_diff / abs(angle_diff)) * slow_turn_speed
                self.send_cmd(linear, angular)
            else:
                linear = forward_speed
                angular = angle_diff * 0.3
                self.send_cmd(linear, angular)

            self.rate.sleep()

    def send_cmd(self, linear, angular):
        base = linear
        turn = angular * 0.6

        cmd = Drive()
        cmd.left = max(min(base - turn, 1.0), -1.0)
        cmd.right = max(min(base + turn, 1.0), -1.0)

        if abs(linear) < 0.01 and abs(angular) < 0.01:
            cmd.left = 0.0
            cmd.right = 0.0

        self.pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        WaypointNavigator()
    except rospy.ROSInterruptException:
        pass

