#!/usr/bin/env python3
"""Send the AGV to a goal (or through several waypoints) on the saved map.

Run in a terminal with ROS sourced (the YOLO venv is NOT needed):
  python3 goto_goal.py 3.0 -2.0 90                  # x [m]  y [m]  yaw [deg]
  python3 goto_goal.py --waypoints "4,-3,0; -6,6,90; 0,0,0"
Options: --timeout SECONDS (default 600)   --no-wait (skip the Nav2-ready check)
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def parse_waypoints(text):
    out = []
    for chunk in text.split(';'):
        chunk = chunk.strip()
        if chunk:
            vals = [float(v) for v in chunk.split(',')]
            out.append((vals[0], vals[1], vals[2] if len(vals) > 2 else 0.0))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('goal', nargs='*', type=float, help='x y [yaw_deg]')
    ap.add_argument('--waypoints', help='"x,y,yaw; x,y,yaw; ..." visited in order')
    ap.add_argument('--timeout', type=float, default=600.0, help='give up after this many seconds')
    ap.add_argument('--no-wait', action='store_true', help='do not wait for Nav2 to report active')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    if args.waypoints:
        goals = parse_waypoints(args.waypoints)
    elif len(args.goal) >= 2:
        goals = [(args.goal[0], args.goal[1], args.goal[2] if len(args.goal) > 2 else 0.0)]
    else:
        ap.error('give a goal "x y [yaw_deg]" or --waypoints')

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    if not args.no_wait:
        nav.waitUntilNav2Active()

    t0 = time.time()
    last_print = 0.0
    if len(goals) == 1:
        nav.goToPose(make_pose(nav, *goals[0]))
    else:
        nav.followWaypoints([make_pose(nav, *g) for g in goals])
    print(f'Goal(s): {goals}')

    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and time.time() - last_print > 2.0:
            last_print = time.time()
            if hasattr(fb, 'distance_remaining'):
                print(f'  distance remaining: {fb.distance_remaining:.2f} m')
            elif hasattr(fb, 'current_waypoint'):
                print(f'  heading to waypoint {fb.current_waypoint + 1} of {len(goals)}')
        if time.time() - t0 > args.timeout:
            print('Timeout: cancelling')
            nav.cancelTask()
            break

    result = nav.getResult()
    print({TaskResult.SUCCEEDED: 'RESULT: goal reached',
           TaskResult.CANCELED: 'RESULT: cancelled',
           TaskResult.FAILED: 'RESULT: FAILED (see the Nav2 terminal for the reason)'}.get(result, f'RESULT: {result}'))
    nav.lifecycleShutdown() if False else None   # leave Nav2 running
    rclpy.shutdown()


if __name__ == '__main__':
    main()
