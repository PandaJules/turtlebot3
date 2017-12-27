#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
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


def filter_front_scan(laser_scan):
    f = laser_scan[-90:]+laser_scan[:91]
    return list(map(lambda dist: dist if not np.isinf(dist) else max_range,
                    f[::-1]))


def take_one_front_shot():
    front = filter_front_scan(scan)
    # front = [random.uniform(0, 4) for _ in range(181)]
    angles = range(-90, 91)
    plt.figure()
    plt.plot(angles, front)
    plt.xlim((-90, 90))
    filename = "/home/julia/Desktop/scan"
    i = 0
    while os.path.exists('{}{:d}.png'.format(filename, i)):
        i += 1
    plt.savefig('{}{:d}.png'.format(filename, i))


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
                take_one_front_shot()
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
