#!/usr/bin/env python

import numpy as np
import random

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from math import degrees, radians
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Int32, Bool, Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os

""""**************************
* Globals
***************************"""
PI = 3.14159265
half_wheel_separation = 0.07
rotate_angle = 30
angles = [90, 45, 0, -45, -90]
front_distance_limit = 0.9
oblique_distance_limit = 0.5
side_distance_limit = 0.4
# 1.2, 0.6, 0.4
wheel_radius = 0.03
right_joint_encoder = 0.0
new_right_joint_encoder = 0.0
direction_vector = [1, 5, 5, 5, 1]
# States for FSM
GET_DIRECTION = 0
DRIVE_FORWARD = 1
LEFT_TURN = 2
RIGHT_TURN = 3
# 5 sensors to the WEST, NW, N, NE, EAST
LEFT = 0
NW = 1
CENTER = 2
NE = 3
RIGHT = 4
# State can be one of get_direction, driving forward, turning left or right
turtlebot3_state = 1
turtlebot3_lin_vel = 0
turtlebot3_ang_vel = 0
theta = 0
t = 0
x_sim, y_sim = 0, 0
prev_lap = 0
trying_to_stabilize = False


def odomMsgCallBack(odom_msg):
    global x_sim, y_sim, theta
    x_sim = odom_msg.pose.pose.position.x
    y_sim = odom_msg.pose.pose.position.y

    quaternion = odom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([quaternion.x,
                                                  quaternion.y,
                                                  quaternion.z,
                                                  quaternion.w])

    theta = degrees(theta % (2 * PI))


def jointStateMsgCallBack(joint_state_msg):
    global right_joint_encoder
    right_joint_encoder = joint_state_msg.position[0]


def lapMsgCallBack(lap_msg):
    global prev_lap, trying_to_stabilize
    cur_lap = lap_msg.data
    if cur_lap != prev_lap:
        if cur_lap == 0:
            pass
        prev_lap = cur_lap
        trying_to_stabilize = True
    else:
        trying_to_stabilize = False


def stabilize():
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0

    model_state_msg = ModelState()
    model_state_msg.model_name = 'turtlebot3_burger'
    # yaw = random.uniform(-PI / 4, PI / 4)
    yaw = 0
    [model_state_msg.pose.orientation.x,
     model_state_msg.pose.orientation.y,
     model_state_msg.pose.orientation.z,
     model_state_msg.pose.orientation.w] = quaternion_from_euler(0, 0, yaw)
    if rospy.has_param('startXY'):
        x = rospy.get_param('startXY/x')
        y = rospy.get_param('startXY/y')
    else:
        rospy.logerr("NO PARAMETER NAMED startXY, setting to default")
        x = 0
        y = 0
    model_state_msg.pose.position.x = x
    model_state_msg.pose.position.y = y
    model_state_msg.pose.position.z = 0

    while not (turtlebot3_lin_vel == 0 and turtlebot3_ang_vel == 0 and x_sim == x and y_sim == y):
        cmd_vel_pub.publish(cmd_vel)
        model_pub.publish(model_state_msg)
        wait_traj.publish(Bool(data=False))

    rospy.sleep(2)

    while not (turtlebot3_lin_vel == 0 and turtlebot3_ang_vel == 0 and x_sim == x and y_sim == y):
        cmd_vel_pub.publish(cmd_vel)
        model_pub.publish(model_state_msg)
        wait_traj.publish(Bool(data=False))


def cmdMsgCallBack(cmd_msg):
    global turtlebot3_lin_vel, turtlebot3_ang_vel
    turtlebot3_lin_vel = cmd_msg.linear.x
    turtlebot3_ang_vel = cmd_msg.angular.z


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


def updateCommandVelocity(linear, angular):
    cmd_vel = Twist()
    cmd_vel.linear.x = linear
    cmd_vel.angular.z = angular
    cmd_vel_pub.publish(cmd_vel)


def rotate_with_odometry(angle, velocity_publisher, clockwise=False, angular_speed=0.25):
    initial_orientation = theta
    relative_angle = angle  # * PI / 180

    vel_msg = Twist()
    if clockwise:
        goal_angle = (initial_orientation - relative_angle) % 360
        vel_msg.angular.z = -abs(angular_speed)
    else:
        goal_angle = (initial_orientation + relative_angle) % 360
        vel_msg.angular.z = abs(angular_speed)

    while abs(theta - goal_angle) > 0.5:
        print(theta, goal_angle, theta - goal_angle)
        velocity_publisher.publish(vel_msg)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print("Rotated successfully")
    print("Rotation was by {}".format(abs(theta - initial_orientation)))
    print("New orientation is: ", theta)


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    global new_right_joint_encoder, turtlebot3_state, t
    wait_traj.publish(Bool(data=True))
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.22
    rot_vel_left = Twist()
    rot_vel_left.angular.z = 1.57
    rot_vel_right = Twist()
    rot_vel_right.angular.z = -1.57

    if turtlebot3_state == GET_DIRECTION:
        """Normally TURN RIGHT, unless there is a wall, then either TURN LEFT or GO FORWARD"""
        print("GETTING DIRECTION\n")
        if direction_vector[CENTER] < front_distance_limit:
            if direction_vector[RIGHT] < side_distance_limit < direction_vector[LEFT] or \
                    direction_vector[NE] < oblique_distance_limit and direction_vector[RIGHT] < side_distance_limit and \
                    direction_vector[NW] > oblique_distance_limit or \
                    direction_vector[NW] > oblique_distance_limit and direction_vector[LEFT] > side_distance_limit and \
                    direction_vector[NE] < oblique_distance_limit:
                print([direction_vector[LEFT] < side_distance_limit, direction_vector[NW] < oblique_distance_limit,
                       direction_vector[CENTER], direction_vector[NE] < oblique_distance_limit,
                       direction_vector[RIGHT] < side_distance_limit])
                turtlebot3_state = LEFT_TURN
            else:
                print([direction_vector[LEFT] < side_distance_limit, direction_vector[NW] < oblique_distance_limit,
                       direction_vector[CENTER], direction_vector[NE] < oblique_distance_limit,
                       direction_vector[RIGHT] < side_distance_limit])
                turtlebot3_state = RIGHT_TURN

        else:
            if direction_vector[NW] < oblique_distance_limit < direction_vector[NE] and \
                    direction_vector[RIGHT] > side_distance_limit:
                print([direction_vector[LEFT] < side_distance_limit, direction_vector[NW] < oblique_distance_limit,
                       direction_vector[CENTER], direction_vector[NE] < oblique_distance_limit,
                       direction_vector[RIGHT] < side_distance_limit])
                turtlebot3_state = RIGHT_TURN
            elif direction_vector[NE] < oblique_distance_limit < direction_vector[NW] and \
                    direction_vector[LEFT] > side_distance_limit:
                print([direction_vector[LEFT] < side_distance_limit, direction_vector[NW] < oblique_distance_limit,
                       direction_vector[CENTER], direction_vector[NE] < oblique_distance_limit,
                       direction_vector[RIGHT] < side_distance_limit])
                turtlebot3_state = LEFT_TURN
            else:
                turtlebot3_state = DRIVE_FORWARD

    elif turtlebot3_state == DRIVE_FORWARD:
        print("FORWARD\n")
        cmd_vel_pub.publish(cmd_vel)
        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == LEFT_TURN:
        print("turning LEFT\n")
        rotate_with_odometry(rotate_angle, cmd_vel_pub, clockwise=False)
        turtlebot3_state = GET_DIRECTION

    elif turtlebot3_state == RIGHT_TURN:
        print("turning RIGHT\n")
        rotate_with_odometry(rotate_angle, cmd_vel_pub, clockwise=True)
        turtlebot3_state = GET_DIRECTION

    else:
        turtlebot3_state = GET_DIRECTION


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('Rule_based_turtlebot3')
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmdMsgCallBack)
    wait_traj = rospy.Publisher('/can_log', Bool, queue_size=3)
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=2)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)
    joint_state_sub = rospy.Subscriber('/joint_states', JointState, jointStateMsgCallBack)
    odometry_sub = rospy.Subscriber('/odom', Odometry, odomMsgCallBack)
    laps_sub = rospy.Subscriber('/laps', Int32, lapMsgCallBack)

    r = rospy.Rate(250)
    r.sleep()
    while not rospy.is_shutdown():
        if trying_to_stabilize:
            stabilize()
            trying_to_stabilize = False
        else:
            controlLoop()
            r.sleep()
