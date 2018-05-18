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
MAX_TB3_LINEAR = 0.7
MAX_TB3_ANGULAR = 1.7
direction_vector = [0, 0, 0, 0, 0]
angles = [90, 45, 0, -45, -90]
# 5 sensors to the WEST, NW, N, NE, EAST
LEFT = 0
NW = 1
CENTER = 2
NE = 3
RIGHT = 4

a = 0.1
b = 0.2
c = 0.15

# NN weights 5x2 for every sensor to each wheel
WEIGHTS = [[0, 0, a, b, c],
           [c, b, a, 0, 0]]
V_bias = [0.2, 0.2]


def laserScanMsgCallBack(laser_msg):
    global direction_vector
    scan = laser_msg.ranges
    for counter, angle in enumerate(angles):
        if np.isinf(scan[angle]):
            direction_vector[counter] = laser_msg.range_max
        else:
            direction_vector[counter] = np.median([scan[angle - 2], scan[angle - 1],
                                                   scan[angle], scan[angle + 1],
                                                   scan[angle + 2]])


def set_wheel_velocities(left_wheel_speed=0.0, right_wheel_speed=0.0):
    global V_bias
    cmd_vel = Twist()

    # set linear forward velocity
    cmd_vel.linear.x = (right_wheel_speed + left_wheel_speed) / 2.0
    if cmd_vel.linear.x > MAX_TB3_LINEAR:
        V_bias = [max(v - 0.05, -0.5) for v in V_bias]
    elif cmd_vel.linear.x < 0.2:
        V_bias = [min(v + 0.05, 1.0) for v in V_bias]

    # set angular velocity
    cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) / (2 * half_wheel_separation)
    if cmd_vel.angular.z > 0:
        cmd_vel.angular.z = min(MAX_TB3_ANGULAR, cmd_vel.angular.z)
    else:
        cmd_vel.angular.z = max(-MAX_TB3_ANGULAR, cmd_vel.angular.z)

    cmd_vel_pub.publish(cmd_vel)


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    # Velocities of L and R wheels are cross-connected to RHS and LHS sensors
    left_vel = np.dot(WEIGHTS[0], direction_vector) + V_bias[0]
    right_vel = np.dot(WEIGHTS[1], direction_vector) + V_bias[1]
    set_wheel_velocities(left_vel.item(), right_vel.item())


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('Neural_net_turtlebot3')
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()
