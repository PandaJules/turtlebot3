#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from tf.transformations import euler_from_quaternion
import os

""""**************************
* Globals
***************************"""

PI = 3.14159265
half_wheel_separation = 0.08
rotate_angle = 45.0 * PI / 180
front_distance_limit = 0.7
side_distance_limit = 0.4
wheel_radius = 0.033
right_joint_encoder = 0.0
priv_right_joint_encoder = 0.0
direction_vector = [0]*5
# States for FSM
GET_DIRECTION = 0
DRIVE_FORWARD = 1
RIGHT_TURN = 2
LEFT_TURN = 3
# 5 sensors to the WEST, NW, N, NE, EAST
CENTER = 0
NW = 1
LEFT = 2
RIGHT = 3
NE = 4
# State can be one of get_direction, driving forward, turning left or right
turtlebot3_state = 0


def jointStateMsgCallBack(joint_state_msg):
    global right_joint_encoder
    right_joint_encoder = joint_state_msg.position[0]


def laserScanMsgCallBack(laser_msg):
    global direction_vector
    angles = [0, 45, 90, -90, -45]

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


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    global priv_right_joint_encoder, turtlebot3_state
    wheel_rotation_angle = (rotate_angle * half_wheel_separation / wheel_radius)
    linear_vel_x = 0.3
    angular_vel_z = 1.5

    if turtlebot3_state == GET_DIRECTION:
        """Normally TURN RIGHT, unless there is a wall, then either TURN LEFT or GO FORWARD"""
        if direction_vector[LEFT] < side_distance_limit:
            if direction_vector[NW] > side_distance_limit > direction_vector[NE]:
                priv_right_joint_encoder = right_joint_encoder + wheel_rotation_angle
                turtlebot3_state = LEFT_TURN
            elif direction_vector[CENTER] < front_distance_limit:
                priv_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
                turtlebot3_state = RIGHT_TURN
            else:
                turtlebot3_state = DRIVE_FORWARD

        else:
            if direction_vector[NE] > side_distance_limit > direction_vector[NW] \
                    and direction_vector[CENTER] < front_distance_limit:
                priv_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
                turtlebot3_state = RIGHT_TURN
            elif direction_vector[CENTER] > front_distance_limit:
                turtlebot3_state = DRIVE_FORWARD
            else:
                priv_right_joint_encoder = right_joint_encoder + wheel_rotation_angle
                turtlebot3_state = LEFT_TURN

    elif turtlebot3_state == DRIVE_FORWARD:
        updateCommandVelocity(linear_vel_x, 0.0)
        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == RIGHT_TURN:
        if abs(priv_right_joint_encoder - right_joint_encoder) < 0.1:
            turtlebot3_state = GET_DIRECTION
        else:
            updateCommandVelocity(0.0, -1 * angular_vel_z)

    elif turtlebot3_state == LEFT_TURN:
        if abs(priv_right_joint_encoder - right_joint_encoder) < 0.1:
                turtlebot3_state = GET_DIRECTION
        else:
            updateCommandVelocity(0.0, angular_vel_z)

    else:
        turtlebot3_state = GET_DIRECTION


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    log_path = os.path.join(os.path.expanduser('~'), "Desktop")

    rospy.init_node('ros_gazebo_turtlebot3')
    rospy.loginfo("robot_model : BURGER")
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)
    joint_state_sub = rospy.Subscriber('/joint_states', JointState, jointStateMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()

