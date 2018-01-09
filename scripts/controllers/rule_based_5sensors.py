#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import degrees, radians
from sensor_msgs.msg import LaserScan, JointState
from tf.transformations import euler_from_quaternion
import os

""""**************************
* Globals
***************************"""

PI = 3.14159265
half_wheel_separation = 0.07
rotate_angle = radians(45)
front_distance_limit = 0.8
oblique_distance_limit = 0.6
side_distance_limit = 0.5
wheel_radius = 0.03
right_joint_encoder = 0.0
new_right_joint_encoder = 0.0
direction_vector = [0]*5
# States for FSM
GET_DIRECTION = 0
DRIVE_FORWARD = 1
LEFT_TURN = 2
RIGHT_TURN = 3
# 5 sensors to the WEST, NW, N, NE, EAST
CENTER = 0
NW = 1
LEFT = 2
RIGHT = 3
NE = 4
# State can be one of get_direction, driving forward, turning left or right
turtlebot3_state = 0
turtlebot3_lin_vel = 0
turtlebot3_ang_vel = 0
theta = 0


def jointStateMsgCallBack(joint_state_msg):
    global right_joint_encoder
    right_joint_encoder = joint_state_msg.position[0]


def laserScanMsgCallBack(laser_msg):
    global direction_vector, dv
    angles = [0, 45, 90, -90, -45]

    for counter, angle in enumerate(angles):
        if np.isinf(laser_msg.ranges[angle]):
            direction_vector[counter] = laser_msg.range_max
        else:
            direction_vector[counter] = laser_msg.ranges[angle]
    # dv[0] = direction_vector[2] < front_distance_limit
    # dv[1] = direction_vector[1] < oblique_distance_limit
    # dv[2] = direction_vector[0] < side_distance_limit
    # dv[3] = direction_vector[4] < oblique_distance_limit
    # dv[4] = direction_vector[3] < side_distance_limit


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


def cmdMsgCallBack(cmd_msg):
    global turtlebot3_lin_vel, turtlebot3_ang_vel
    turtlebot3_lin_vel = cmd_msg.linear.x
    turtlebot3_ang_vel = cmd_msg.angular.z


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    global new_right_joint_encoder, turtlebot3_state
    wheel_rotation_angle = 1.01*(rotate_angle * half_wheel_separation / wheel_radius)
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.25
    rot_vel_left = Twist()
    rot_vel_left.angular.z = 1.57
    rot_vel_right = Twist()
    rot_vel_right.angular.z = -1.57
    t=0

    if turtlebot3_state == GET_DIRECTION:
        """Normally TURN RIGHT, unless there is a wall, then either TURN LEFT or GO FORWARD"""
        # if direction_vector[LEFT] < side_distance_limit:
        #     if direction_vector[NW] > side_distance_limit > direction_vector[NE]:
        #         new_right_joint_encoder = right_joint_encoder + wheel_rotation_angle
        #         turtlebot3_state = LEFT_TURN
        #     elif direction_vector[CENTER] < front_distance_limit:
        #         new_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
        #         turtlebot3_state = RIGHT_TURN
        #     else:
        #         turtlebot3_state = DRIVE_FORWARD
        #
        # else:
        #     if direction_vector[NE] > side_distance_limit > direction_vector[NW] \
        #             and direction_vector[CENTER] < front_distance_limit:
        #         new_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
        #         turtlebot3_state = RIGHT_TURN
        #     elif direction_vector[CENTER] > front_distance_limit:
        #         turtlebot3_state = DRIVE_FORWARD
        #     else:
        #         new_right_joint_encoder = right_joint_encoder + wheel_rotation_angle
        #         turtlebot3_state = LEFT_TURN

        if direction_vector[LEFT] < side_distance_limit:
            if direction_vector[NW] > oblique_distance_limit > direction_vector[NE]:
                turtlebot3_state = LEFT_TURN
            elif direction_vector[CENTER] < front_distance_limit or direction_vector[NW] < oblique_distance_limit:
                t = degrees(theta % (2 * PI))
                turtlebot3_state = RIGHT_TURN
            else:
                turtlebot3_state = DRIVE_FORWARD

        else:
            if direction_vector[NE] < oblique_distance_limit and \
                    (direction_vector[NW] > oblique_distance_limit or direction_vector[CENTER] < front_distance_limit):
                turtlebot3_state = LEFT_TURN
            elif direction_vector[CENTER] > front_distance_limit and direction_vector[NW] > oblique_distance_limit:
                turtlebot3_state = DRIVE_FORWARD
            else:
                t = degrees(theta % (2 * PI))
                turtlebot3_state = RIGHT_TURN

    elif turtlebot3_state == DRIVE_FORWARD:
        cmd_vel_pub.publish(cmd_vel)
        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == LEFT_TURN:
        new_right_joint_encoder = right_joint_encoder + wheel_rotation_angle
        while abs(new_right_joint_encoder - right_joint_encoder) > 0.05:
            cmd_vel_pub.publish(rot_vel_left)

        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == RIGHT_TURN:
        new_right_joint_encoder = right_joint_encoder - wheel_rotation_angle
        t = degrees(theta % (2 * PI))
        a = right_joint_encoder
        while abs(new_right_joint_encoder - right_joint_encoder) > 0.05:
            cmd_vel_pub.publish(rot_vel_right)
        print(a - right_joint_encoder)
        print(abs(degrees(theta % (2 * PI)) - t))
        turtlebot3_state = GET_DIRECTION

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

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmdMsgCallBack)

    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)
    joint_state_sub = rospy.Subscriber('/joint_states', JointState, jointStateMsgCallBack)
    odometry_sub = rospy.Subscriber('/odom', Odometry, update_angle)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        controlLoop()
        r.sleep()

