#!/usr/bin/env python
"""
Tele-operation of the Turtlebot using the keyboard
Pressing space takes a shot of lidar data and outputs distances for 360 degrees as a plot
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import sys, select, termios, tty, os
import matplotlib.pyplot as plt
scan = []
max_range = 50


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        k = sys.stdin.read(1)
    else:
        k = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return k


def update_laser_scan(scan_msg):
    """scan_msg contains a list of 360 elements, each representing distance for 1 angle"""
    global scan, max_range
    scan = scan_msg.ranges
    max_range = scan_msg.range_max


def filter_front_scan(laser_scan, start_angle=-90, stop_angle=91):
    f = laser_scan[start_angle:]+laser_scan[:stop_angle]
    return list(map(lambda dist: dist if not np.isinf(dist) else max_range,
                    f[::-1]))


def take_front_shot(start_angle=-90, stop_angle=91, new_file=False, multiple=False):
    front = filter_front_scan(scan, start_angle=start_angle, stop_angle=stop_angle)
    angles = range(start_angle, stop_angle)
    if not multiple:
        plt.figure()
    plt.plot(angles, front)
    plt.xlim((start_angle, stop_angle-1))
    filename = os.path.join(os.path.expanduser('~'), "Desktop")+"/scan"
    if new_file:
        i = 0
        while os.path.exists('{}{:d}.png'.format(filename, i)):
            i += 1
        plt.savefig('{}{:d}.png'.format(filename, i))
    else:
        plt.savefig('{}__.png'.format(filename))


if __name__ == "__main__":

    rospy.init_node('noise_analyser')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    sub1 = rospy.Subscriber('/scan', LaserScan, update_laser_scan)

    r = rospy.Rate(5)

    settings = termios.tcgetattr(sys.stdin)

    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0
    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'w':
                target_linear_vel = target_linear_vel + 0.01
            elif key == 'x':
                target_linear_vel = target_linear_vel - 0.01
            elif key == 'a':
                target_angular_vel = target_angular_vel + 0.1
            elif key == 'd':
                target_angular_vel = target_angular_vel - 0.1
            elif key == 's':
                target_linear_vel = 0
                control_linear_vel = 0
                target_angular_vel = 0
                control_angular_vel = 0
            elif key == ' ':
                take_front_shot(start_angle=-10, stop_angle=11, multiple=True)
            else:
                if key == '\x03':
                    break

            if target_linear_vel > control_linear_vel:
                control_linear_vel = min(target_linear_vel, control_linear_vel + (0.01 / 4.0))
            else:
                control_linear_vel = target_linear_vel

            if target_angular_vel > control_angular_vel:
                control_angular_vel = min(target_angular_vel, control_angular_vel + (0.1 / 4.0))
            else:
                control_angular_vel = target_angular_vel

            twist = Twist()
            twist.linear.x = control_linear_vel
            twist.angular.z = control_angular_vel
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
