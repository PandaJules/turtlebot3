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
direction_vector = [0, 0, 0, 0, 0]
# 5 sensors to the WEST, NW, N, NE, EAST
LEFT = 0
NW = 1
CENTER = 2
NE = 3
RIGHT = 4

# NN weights 5x2 for every sensor to each wheel
WEIGHTS = [[0, 0, 0.1, 0.3, 0.15],
           [0.15, 0.3, 0.1, 0, 0]]
V_bias = [0.4, 0.4]


def laserScanMsgCallBack(laser_msg):
    global direction_vector
    scan = laser_msg.ranges
    angles = [89, 44, 0, -45, -90]

    for counter, angle in enumerate(angles):
        if np.isinf(scan[angle]):
            direction_vector[counter] = laser_msg.range_max
        else:
            direction_vector[counter] = scan[angle]


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
    # Velocities of L and R wheels are cross-connected to RHS and LHS sensors
    left_vel = np.dot(WEIGHTS[0], direction_vector) + V_bias[0]
    right_vel = np.dot(WEIGHTS[1], direction_vector) + V_bias[1]
    set_wheel_velocities(left_vel.item(), right_vel.item())


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('ros_gazebo_turtlebot3')
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()
