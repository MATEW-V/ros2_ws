#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        #PID var 
        self.declare_parameter('kp', 1.0) 
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.06)

        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('lookahead_distance', 1.0)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.desired_distance = float(
            self.get_parameter('desired_distance').value
        )

        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_wall_error = 0.0

        self.prev_time = self.get_clock().now()

        self.prev_steering = 0.0
        self.prev_b = self.desired_distance

        self.integral_limit = 1.0
        self.max_steering = 0.34
        self.smoothing_alpha = 0.5

        # Left wall:
        # B = 90 degrees
        # A = 50 degrees
        self.theta = np.radians(40.0) #gap between 2 lidar
        self.angle_b = np.radians(90.0)
        self.angle_a = self.angle_b - self.theta

        self.forward_angle = 0.0 
        self.max_hallway_distance = 2.0 #just a guess idfk

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        self.get_logger().info(
            'Wall Following Node Initialized'
        )

    def get_robust_range( #21 rays with 90deg in middle, 10 each side
        self,
        scan,
        center_angle,
        window=21
    ):
        center_index = int(
            round(
                (center_angle - scan.angle_min) /
                scan.angle_increment
            )
        )
        valid_ranges = []
        half_window = window // 2 #doing the 10 ray a side wit int div
        for i in range(
            -half_window,
            half_window + 1
        ):
            idx = center_index + i
            if 0 <= idx < len(scan.ranges):
                val = scan.ranges[idx]
                if (
                    np.isfinite(val)
                    and scan.range_min <= val <= scan.range_max
                ):
                    valid_ranges.append(val)
        if valid_ranges:
            return float(np.median(valid_ranges))
        return float(scan.range_max)
    
    def calc_error(self, scan):
        #left-side measurements
        b = self.get_robust_range(
            scan, self.angle_b, window=21
        )
        a = self.get_robust_range(
            scan, self.angle_a, window=21
        )

        #forward-facing measurement
        forward = self.get_robust_range(
            scan, 0.0, window=11
        )

        if (
            not np.isfinite(a)
            or not np.isfinite(b)
            or a <= 0.0
            or b <= 0.0
        ):
            return self.prev_wall_error

        #stupid ass opening fix near the back of default map looking like --====-- 
        sudden_opening = (
            b > self.max_hallway_distance
            or (b - self.prev_b) > 0.5
        )

        if sudden_opening:
            if forward > 1.5:
                return self.prev_wall_error

        #the pid math tan orientation part
        numerator = (
            a * np.cos(self.theta) - b
        )
        denominator = (
            a * np.sin(self.theta)
        )

        if abs(denominator) < 1e-6:
            return self.prev_wall_error
        alpha = np.arctan2(
            numerator,
            denominator
        )

        d_current = b * np.cos(alpha)
        d_future = (d_current + self.lookahead_distance * np.sin(alpha))

        error = d_future - self.desired_distance

        self.prev_b = b
        self.prev_wall_error = error

        return float(error)

    #pided it
    def pid_control(self, error):

        current_time = self.get_clock().now()

        dt = (current_time - self.prev_time).nanoseconds / 1e9

        dt = float(np.clip(dt, 0.001, 0.1))

        self.integral += error * dt

        self.integral = float(
            np.clip(
                self.integral,
                -self.integral_limit,
                self.integral_limit
            )
        )
        
        derivative = (
            error - self.prev_error
        ) / dt

        self.prev_error = error
        self.prev_time = current_time

        raw_steering = (self.kp * error + self.ki * self.integral + self.kd * derivative)

        # limit steering angle
        raw_steering = float(
            np.clip(
                raw_steering,
                -self.max_steering,
                self.max_steering
            )
        )

        steering_angle = (
            self.smoothing_alpha * raw_steering
            + (1.0 - self.smoothing_alpha)
            * self.prev_steering
        )
        self.prev_steering = steering_angle
        return float(steering_angle)

    def calculate_speed(self, steering_angle):
        abs_steer = abs(steering_angle)
        spd_mult = 1.0 #for fun and testing higher speeds

        if abs_steer < np.radians(10):
            speed = 1.5 * spd_mult
        elif abs_steer < np.radians(20):
            speed = 1.0 * spd_mult
        else:
            speed = 0.5 * spd_mult
        return speed

    def laser_callback(self, msg):
        error = self.calc_error(msg)
        steering_angle = self.pid_control(error)
        speed = self.calculate_speed(steering_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = (self.get_clock().now().to_msg())
        drive_msg.header.frame_id = 'laser'
        drive_msg.drive.steering_angle = (steering_angle)
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a stop command before shutting down
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = (
            node.get_clock().now().to_msg()
        )
        stop_msg.header.frame_id = 'laser'
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.drive_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()