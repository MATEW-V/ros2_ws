#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyNode(Node):
    #The class that handles emergency braking.
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.odom_sub = self.create_subscription(
            Odometry, #msg type
            '/ego_racecar/odom', #topic
            self.odom_callback, #callback
            10 #queue size? idk
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        #ttc is time to collision yea so like (range mesure / spd*cos(angle))
        # range mesure is a fat array apparently and everything else so numpy good
        ranges = np.array(scan_msg.ranges)
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment

        vcostheta=self.speed*np.cos(angles)
        filter = ((vcostheta>0) & (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)) 

        if not np.any(filter):
            return

        ittcArr = (ranges[filter]/vcostheta[filter])
        minITTC = np.min(ittcArr)

        # TODO: publish command to brake
        #self.get_logger().info(f"Speed: {self.speed:.2f}m/s | current min iTTC: {minITTC:.3f}s")
        if minITTC < 0.4:
            self.get_logger().warn(f"breaked woooomin ittc: {minITTC:.3f}s")
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()