#!/usr/bin/env python3
"""Measure how much the odometry over-/under-reports in-place rotation, and suggest a
new <wheel_separation> for the DiffDrive plugin.

Prerequisites: simulation + Nav2 running and the robot localized (AMCL), robot in open space.
  python3 calibrate_rotation.py                       # 1 turn at 0.4 rad/s, current separation 0.56
  python3 calibrate_rotation.py --separation 0.78 --turns 2

How it works: the robot is turned on the spot until the ODOMETRY says it has turned N times.
The REAL turn is read from the lidar localization (map->base_link).
  new_separation = current_separation * (odom_turn / real_turn)
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def yaw_of(tf):
    q = tf.transform.rotation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Calib(Node):
    def __init__(self, turns, speed, sep):
        super().__init__('rotation_calibration')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.target = 2 * math.pi * turns
        self.speed, self.sep = speed, sep
        self.last = None
        self.acc_odom = 0.0
        self.acc_map = 0.0
        self.state = 'turn'
        self.t_settle = None
        self.done = False
        self.create_timer(0.1, self.tick)

    def read(self):
        try:
            o = yaw_of(self.buf.lookup_transform('odom', 'base_link', rclpy.time.Time()))
            m = yaw_of(self.buf.lookup_transform('map', 'base_link', rclpy.time.Time()))
            return o, m
        except Exception:
            return None

    def tick(self):
        cur = self.read()
        if cur is None:
            self.get_logger().info('waiting for TF odom->base_link and map->base_link '
                                   '(is Nav2 / AMCL running?)', throttle_duration_sec=3.0)
            return
        if self.last is not None:
            self.acc_odom += wrap(cur[0] - self.last[0])
            self.acc_map += wrap(cur[1] - self.last[1])
        self.last = cur

        if self.state == 'turn':
            if abs(self.acc_odom) < self.target:
                t = Twist()
                t.angular.z = self.speed
                self.pub.publish(t)
            else:
                self.pub.publish(Twist())
                self.state = 'settle'
                self.t_settle = self.get_clock().now()
        elif self.state == 'settle':
            self.pub.publish(Twist())
            if (self.get_clock().now() - self.t_settle).nanoseconds * 1e-9 > 5.0:
                self.report()
                self.done = True

    def report(self):
        o, m = abs(self.acc_odom), abs(self.acc_map)
        print(f'\nodometry turned : {math.degrees(o):7.1f} deg')
        print(f'real (lidar)    : {math.degrees(m):7.1f} deg')
        if m < math.radians(30):
            print('Real rotation is tiny: the wheels spin but the robot barely turns '
                  '(friction / torque problem), or AMCL is not localized. Do not trust a ratio yet.')
            return
        ratio = o / m
        print(f'ratio odom/real : {ratio:.3f}')
        print(f'suggested <wheel_separation>: {self.sep * ratio:.3f}  (currently {self.sep})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=float, default=1.0)
    ap.add_argument('--speed', type=float, default=0.4, help='rad/s')
    ap.add_argument('--separation', type=float, default=0.56, help='current <wheel_separation>')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Calib(args.turns, args.speed, args.separation)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
