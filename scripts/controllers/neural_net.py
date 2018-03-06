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
c = 0.3
WEIGHTS = [[0, 0, c/3, c, c/2],
           [c/2, c, c/3, 0, 0]]
V_bias = [-0.3, -0.3]


def laserScanMsgCallBack(laser_msg):
    global direction_vector
    scan = laser_msg.ranges
    angles = [90, 45, 0, -45, -90]

    for counter, angle in enumerate(angles):
        if np.isinf(scan[angle]):
            direction_vector[counter] = 3.5
        else:
            direction_vector[counter] = np.median([scan[angle - 2], scan[angle - 1],
                                                   scan[angle], scan[angle + 1],
                                                   scan[angle + 2]])


def set_wheel_velocities(left_wheel_speed=0.0, right_wheel_speed=0.0):
    global V_bias
    cmd_vel = Twist()
    cmd_vel.linear.x = (right_wheel_speed + left_wheel_speed) / 2.0
    if cmd_vel.linear.x > 0.3:
        V_bias = [v-0.1 for v in V_bias]
    cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) / (2 * half_wheel_separation)
    if cmd_vel.angular.z > 2.8:
        cmd_vel.angular.z = 2.8
    if cmd_vel.angular.z < -2.8:
        cmd_vel.angular.z = -2.8
    cmd_vel_pub.publish(cmd_vel)
    print(cmd_vel.linear.x, cmd_vel.angular.z)


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    # Velocities of L and R wheels are cross-connected to RHS and LHS sensors
    left_vel = np.dot(WEIGHTS[0], direction_vector) #+ V_bias[0]
    right_vel = np.dot(WEIGHTS[1], direction_vector) #+ V_bias[1]
    set_wheel_velocities(left_wheel_speed=left_vel.item(), right_wheel_speed=right_vel.item())


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('ros_gazebo_turtlebot3')
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()
