#!/usr/bin/env python

import numpy as np
import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool, Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler


""""**************************
* Globals
***************************"""

PI = 3.14159265
half_wheel_separation = 0.08
direction_vector = [0, 0, 0, 0, 0]
angles = [90, 45, 0, -45, -90]
MAX_TB3_LINEAR = 0.7
MAX_TB3_ANGULAR = 1.7
# 5 sensors to the WEST, NW, N, NE, EAST
LEFT = 0
NW = 1
CENTER = 2
NE = 3
RIGHT = 4

# NN weights 5x2 for every sensor to each wheel
a = 0.02
b = 0.125
c = 0.1
d = 0.05

WEIGHTS = [[0, 0, d/2, d, d/3],
           [d/3, d, d/2, 0, 0]]
V_bias = [0.3, 0.3]
x_sim, y_sim = 0, 0
prev_lap = 0
trying_to_stabilize = False


def odomMsgCallBack(odom_msg):
    global x_sim, y_sim
    x_sim = odom_msg.pose.pose.position.x
    y_sim = odom_msg.pose.pose.position.y


def lapMsgCallBack(lap_msg):
    global prev_lap, trying_to_stabilize, d, WEIGHTS
    cur_lap = lap_msg.data
    if cur_lap != prev_lap:
        if cur_lap == 0:
            if d == 0.3:
                shut_trajectory.publish(Bool(data=True))
                rospy.signal_shutdown("Search is exhausted")
            else:
                d += 0.05
                WEIGHTS = [[0, 0, d / 2, d, d / 3],
                           [d / 3, d, d / 2, 0, 0]]
        prev_lap = cur_lap
        trying_to_stabilize = True
        shut_trajectory.publish(Bool(data=False))
    else:
        trying_to_stabilize = False
        shut_trajectory.publish(Bool(data=False))


def stabilize():
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0

    model_state_msg = ModelState()
    model_state_msg.model_name = 'turtlebot3_burger'
    yaw = random.uniform(-PI/4, PI/4)
    model_state_msg.pose.orientation=quaternion_from_euler(0,0, yaw)
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

    while not (v1 == 0 and v2 == 0 and x_sim == x and y_sim == y):
        cmd_vel_pub.publish(cmd_vel)
        model_pub.publish(model_state_msg)
        wait_traj.publish(Bool(data=False))

    rospy.sleep(2)

    while not (v1 == 0 and v2 == 0 and x_sim == x and y_sim == y):
        cmd_vel_pub.publish(cmd_vel)
        model_pub.publish(model_state_msg)
        wait_traj.publish(Bool(data=False))


def cmdMsgCallBack(cmd_msg):
    global v1, v2
    v1 = cmd_msg.linear.x
    v2 = cmd_msg.angular.z


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
    # elif cmd_vel.linear.x < 0.2:
    #     V_bias = [min(v + 0.05, 1.0) for v in V_bias]

    # set angular velocity
    cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) / (2 * half_wheel_separation)
    if cmd_vel.angular.z > 0:
        cmd_vel.angular.z = min(MAX_TB3_ANGULAR, cmd_vel.angular.z)
    else:
        cmd_vel.angular.z = max(-MAX_TB3_ANGULAR, cmd_vel.angular.z)

    cmd_vel_pub.publish(cmd_vel)
    print(cmd_vel.linear.x, cmd_vel.angular.z, V_bias)


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    wait_traj.publish(Bool(data=True))
    param_pub.publish(Float64(data=d))
    # Velocities of L and R wheels are cross-connected to RHS and LHS sensors
    left_vel = np.dot(WEIGHTS[0], direction_vector) + V_bias[0]
    right_vel = np.dot(WEIGHTS[1], direction_vector) + V_bias[1]
    set_wheel_velocities(left_wheel_speed=left_vel.item(), right_wheel_speed=right_vel.item())



"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('Neural_net_turtlebot3')
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmdMsgCallBack)
    wait_traj = rospy.Publisher('/can_log', Bool, queue_size=3)
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=2)
    param_pub = rospy.Publisher('/d', Float64, queue_size=2)
    shut_trajectory = rospy.Publisher('/paramSearch', Bool, queue_size=3)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)
    laps_sub = rospy.Subscriber('/laps', Int32, lapMsgCallBack)
    odometry_sub = rospy.Subscriber('/odom', Odometry, odomMsgCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        if trying_to_stabilize:
            stabilize()
            trying_to_stabilize = False
        else:
            controlLoop()
            r.sleep()
