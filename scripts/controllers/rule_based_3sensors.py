#!/usr/bin/env python

import numpy as np
from math import radians, degrees
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from tf.transformations import euler_from_quaternion

""""**************************
* Globals
***************************"""

PI = 3.14159265
half_wheel_separation = 0.07
rotate_angle = radians(45)
front_distance_limit = 0.7
side_distance_limit = 0.4
wheel_radius = 0.03
wheel_rotation_angle = (rotate_angle * half_wheel_separation / wheel_radius)
right_joint_encoder = 0.0
new_right_joint_encoder = 0.0
direction_vector = [0, 0, 0]
GET_DIRECTION = 0
DRIVE_FORWARD = 1
RIGHT_TURN = 2
LEFT_TURN = 3
CENTER = 0
LEFT = 1
RIGHT = 2
turtlebot3_state = 0
theta = 0


def jointStateMsgCallBack(joint_state_msg):
    global right_joint_encoder
    right_joint_encoder = joint_state_msg.position[0]


def laserScanMsgCallBack(laser_msg):
    global direction_vector
    angles = [0, 30, 330]

    for counter, angle in enumerate(angles):
        if np.isinf(laser_msg.ranges[angle]):
            direction_vector[counter] = laser_msg.range_max
        else:
            direction_vector[counter] = laser_msg.ranges[angle]


def updateCommandVelocity(linear, angular):
    cmd_vel = Twist()
    cmd_vel.linear.x = linear
    cmd_vel.angular.z = angular
    cmd_vel_pub.publish(cmd_vel)


def update_angle(odom_msg):
    global theta
    quaternion = odom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([quaternion.x,
                                                  quaternion.y,
                                                  quaternion.z,
                                                  quaternion.w])


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    global new_right_joint_encoder, turtlebot3_state

    cmd_vel = Twist()
    cmd_vel.linear.x = 0.25
    rot_vel_left = Twist()
    rot_vel_left.angular.z = 1.57
    rot_vel_right = Twist()
    rot_vel_right.angular.z = -1.57

    if turtlebot3_state == GET_DIRECTION:
        """Normally TURN RIGHT, unless there is a wall, then either TURN LEFT or GO FORWARD"""
        if direction_vector[LEFT] < side_distance_limit:
            new_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
            turtlebot3_state = RIGHT_TURN
        elif direction_vector[RIGHT] < side_distance_limit:
            new_right_joint_encoder = right_joint_encoder + wheel_rotation_angle
            turtlebot3_state = LEFT_TURN
        elif direction_vector[CENTER] < front_distance_limit:
            new_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
            turtlebot3_state = RIGHT_TURN
        else:
            turtlebot3_state = DRIVE_FORWARD

    elif turtlebot3_state == DRIVE_FORWARD:
        cmd_vel_pub.publish(cmd_vel)
        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == RIGHT_TURN:
        a = right_joint_encoder
        t = degrees(theta%(2*PI))
        while abs(new_right_joint_encoder - right_joint_encoder) > 0.05:
            cmd_vel_pub.publish(rot_vel_right)
        print(a - right_joint_encoder)
        print(abs(degrees(theta%(2*PI))-t))
        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == LEFT_TURN:
        while abs(new_right_joint_encoder - right_joint_encoder) > 0.05:
            cmd_vel_pub.publish(rot_vel_left)

        turtlebot3_state = GET_DIRECTION

    else:
        turtlebot3_state = GET_DIRECTION


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('ros_gazebo_turtlebot3')
    rospy.loginfo("robot_model : BURGER")
    rospy.loginfo("half_wheel_separation : %lf", half_wheel_separation)
    rospy.loginfo("front_distance_limit = %lf", front_distance_limit)
    rospy.loginfo("side_distance_limit = %lf", side_distance_limit)
    rospy.loginfo("To stop TurtleBot CTRL + C")
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)
    odometry_sub = rospy.Subscriber('/odom', Odometry, update_angle)
    joint_state_sub = rospy.Subscriber('/joint_states', JointState, jointStateMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()
