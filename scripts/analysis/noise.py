#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
scan = []
max_range = 50


def update_laser_scan(scan_msg):
    """scan_msg contains a list of 360 elements, each representing distance for 1 angle"""
    global scan, max_range
    scan = scan_msg.ranges
    max_range = scan_msg.range_max


def filter_front_scan(laser_scan):
    return list(map(lambda dist: dist if not np.isinf(dist) else max_range, laser_scan[-90:90]))


def take_one_front_shot():
    front = filter_front_scan(scan)
    angles = range(-90, 91)
    plot(angles, front)


if __name__ == "__main__":

    rospy.init_node('noise_analyser')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    sub1 = rospy.Subscriber('/scan', LaserScan, update_laser_scan)

    r = rospy.Rate(5)

    while not rospy.is_shutdown():
        if key_pressed == Space:
            take_one_front_shot()
        else:
            move_around()

