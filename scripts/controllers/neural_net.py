#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

""""**************************
* Globals
***************************"""

PI = 3.14159265
half_wheel_separation = 0.08
front_distance_limit = 0.7
side_distance_limit = 0.4
sensor_left, sensor_right = 25, 25
direction_vector = [0]*5
# 5 sensors to the WEST, NW, N, NE, EAST
CENTER = 0
NW = 1
LEFT = 2
RIGHT = 3
NE = 4
# NN weights 5x2 for every sensor to each wheel
WEIGHTS = [[1, 1, 1, 0, 0],
           [0, 0, 1, 1, 1]]
V = 0.3
W = 0
V_bias = [0.1, 0.1]
s1 = 0.3
s2 = 0.3


def laserScanMsgCallBack(laser_msg):
    global sensor_right, sensor_left, direction_vector, s1, s2
    scan = laser_msg.ranges
    s1 = scan[29] if not np.isinf(scan[29]) else laser_msg.range_max
    s2 = scan[-30] if not np.isinf(scan[-30]) else laser_msg.range_max
    sensor_left_sum = sum(map(lambda dist: dist if not np.isinf(dist) else laser_msg.range_max, scan[:50]))
    sensor_right_sum = sum(map(lambda dist: dist if not np.isinf(dist) else laser_msg.range_max, scan[-50:]))
    angles = [0, 45, 90, -90, -45]

    for counter, angle in enumerate(angles):
        if np.isinf(scan[angle]):
            direction_vector[counter] = laser_msg.range_max
        else:
            direction_vector[counter] = scan[angle]


def cmdVelCallBack(cmd_msg):
    global V, W
    V = cmd_msg.linear.x
    W = cmd_msg.angular.z


def updateCommandVelocity(linear, angular):
    cmd_vel = Twist()
    cmd_vel.linear.x = linear
    cmd_vel.angular.z = angular
    cmd_vel_pub.publish(cmd_vel)


def set_wheel_velocities(left_wheel_speed=0.0, right_wheel_speed=0.0):
    cmd_vel = Twist()
    cmd_vel.linear.x = (right_wheel_speed + left_wheel_speed)/2.0
    cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed)/(2*half_wheel_separation)
    cmd_vel_pub.publish(cmd_vel)
    print(cmd_vel.linear.x, cmd_vel.angular.z)


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    # left_vel = np.dot(WEIGHTS[0], direction_vector) + V_bias[0]
    # right_vel = np.dot(WEIGHTS[1], direction_vector) + V_bias[1]
    left_vel = 0.3*s2
    right_vel = 0.3*s1
    set_wheel_velocities(left_vel, right_vel)


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('ros_gazebo_turtlebot3')
    rospy.loginfo("robot_model : BURGER")
    rospy.loginfo("turning_radius_ : %lf", half_wheel_separation)
    rospy.loginfo("front_distance_limit_ = %lf", front_distance_limit)
    rospy.loginfo("side_distance_limit_ = %lf", side_distance_limit)
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmdVelCallBack)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()

