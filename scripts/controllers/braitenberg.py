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
V = 0
W = 0


def laserScanMsgCallBack(laser_msg):
    global scan, sensor_right, sensor_left
    scan = laser_msg.ranges
    sensor_left = sum(map(lambda dist: dist if not np.isinf(dist) else laser_msg.range_max, scan[:50]))
    sensor_right = sum(map(lambda dist: dist if not np.isinf(dist) else laser_msg.range_max, scan[-50:]))


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


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    K = 1.0
    update = K*(sensor_right-sensor_left)/(sensor_right+sensor_right)
    left_vel = V*(1 + update)
    right_vel = V*(1 - update)
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

