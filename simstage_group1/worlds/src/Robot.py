#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

class Talkerclass:
    def __init__(self):
        rospy.init_node("talker_node", anonymous=True)

        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.publisher_0 = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/base_scan", LaserScan, self.callback_robot1)
        rospy.Subscriber("/robot_0/base_scan", LaserScan, self.callback_robot0)

        rospy.spin()

    def process_scan(self, scan_data, turn_bias):
        msg = Twist()
        threshold = 1.0  # distance to wall before avoiding
        front_angle = 45  # wider front detection
        side_angle = 80   # wider side detection

        # Check scan is valid
        if not scan_data.ranges or all(np.isnan(scan_data.ranges)):
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # turn slowly to reorient
            return msg

        total_samples = len(scan_data.ranges)
        mid = total_samples // 2
        width_front = int((front_angle / 360.0) * total_samples)
        width_side = int((side_angle / 360.0) * total_samples)

        # Split sectors
        front = scan_data.ranges[mid - width_front : mid + width_front]
        left = scan_data.ranges[mid + width_front : mid + width_front + width_side]
        right = scan_data.ranges[mid - width_front - width_side : mid - width_front]

        # Filter invalid data
        front = [r for r in front if r > 0 and not np.isnan(r) and np.isfinite(r)]
        left = [r for r in left if r > 0 and not np.isnan(r) and np.isfinite(r)]
        right = [r for r in right if r > 0 and not np.isnan(r) and np.isfinite(r)]

        # move forward at slightly random speed
        msg.linear.x = random.uniform(0.15, 0.25)

        # STUCK RECOVERY 
        if front and left and right:
            if min(front) < 0.4 and min(left) < 0.4 and min(right) < 0.4:
                msg.linear.x = -0.2  # reverse
                msg.angular.z = 0.8  # sharp spin
                return msg

        #  OBSTACLE IN FRONT 
        if front and min(front) < threshold:
            avg_left = np.mean(left) if left else 0
            avg_right = np.mean(right) if right else 0
            min_front = min(front)

            # Slow down when too close
            if min_front < 0.6:
                msg.linear.x = 0.1

            # Turn more sharply when closer
            turn_speed = max(0.3, (threshold - min_front) * 1.0)

            # Add more randomness to avoid stuck loops
            turn_jitter = random.uniform(-0.4, 0.4)

            if avg_left > avg_right:
                msg.angular.z = turn_speed + turn_jitter
            else:
                msg.angular.z = -turn_speed - turn_jitter

        else:
            # No obstacle → sweep gently left or right
            msg.angular.z = random.uniform(-0.1, 0.1)

        return msg

    def callback_robot1(self, scan_data):
        try:
            cmd = self.process_scan(scan_data, turn_bias=0.2)
            self.publisher.publish(cmd)
        except Exception as e:
            rospy.logwarn(f"Robot 1 error: {e}")

    def callback_robot0(self, scan_data):
        try:
            cmd = self.process_scan(scan_data, turn_bias=-0.2)
            self.publisher_0.publish(cmd)
        except Exception as e:
            rospy.logwarn(f"Robot 0 error: {e}")

if __name__ == "__main__":
    try:
        Talkerclass()
    except rospy.ROSInterruptException:
        pass
