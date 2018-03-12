#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool
from gazebo_msgs.msg import ContactsState, ModelState
import itertools
import os

""""**************************
* Globals
***************************"""

PI = 3.14159265
half_wheel_separation = 0.08
front_distance_limit = 0.7
side_distance_limit = 0.4
direction_vector = [0, 0, 0, 0, 0]
angles = [90, 45, 0, -45, -90]
MAX_TB3_LINEAR = 0.8
MAX_TB3_ANGULAR = 2.0
# 5 sensors to the WEST, NW, N, NE, EAST
LEFT = 0
NW = 1
CENTER = 2
NE = 3
RIGHT = 4

# NN weights 5x2 for every sensor to each wheel
a = 0.1
b = 0.1
c = 0.2

WEIGHTS = [[0, 0, a, b, c],
           [c, b, a, 0, 0]]
weight_space = [p for p in itertools.product([0.1, 0.2, 0.3, 0.4, 0.5], repeat=3)]
V_bias = [0, 0]
w = 0
lap_num_zeroed = False
bad_weights_file = os.path.join(os.path.expanduser('~'), "Desktop") + "/bad_weights.txt"
trying_to_stabilize = False


def lapMsgCallBack(lap_msg):
    global WEIGHTS, a, b, c, w, lap_num_zeroed
    lap = lap_msg.data
    if lap == 0:
        if not lap_num_zeroed:
            lap_num_zeroed = True
            if w < 125:
                w += 1
                a, b, c = weight_space[w]
                WEIGHTS = [[0, 0, a, b, c],
                           [c, b, a, 0, 0]]
                shut_trajectory.publish(Bool(data=False))
            else:
                print("Weight space is exhausted. Stop the simulation!")
                shut_trajectory.publish(Bool(data=True))
    else:
        lap_num_zeroed = False
        shut_trajectory.publish(Bool(data=False))


def stabilize():
    global WEIGHTS, a, b, c, w
    w += 1
    a, b, c = weight_space[w]
    WEIGHTS = [[0, 0, a, b, c],
               [c, b, a, 0, 0]]
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0

    model_state_msg = ModelState()
    model_state_msg.model_name = 'turtlebot3_burger'
    model_state_msg.pose.orientation.x = 0
    model_state_msg.pose.orientation.y = 0
    model_state_msg.pose.orientation.z = 0
    model_state_msg.pose.orientation.w = 1
    if rospy.has_param('startXY'):
        # x = rospy.get_param('startXY')['x']
        y = rospy.get_param('startXY')
    else:
        rospy.logerr("NO PARAMETER NAMED startXY, setting to default")
        x = 0
        y = 2.5
    model_state_msg.pose.position.x = 0
    model_state_msg.pose.position.y = 2.5
    model_state_msg.pose.position.z = 0

    while (v1, v2) != (0, 0):
        cmd_vel_pub.publish(cmd_vel)
        model_pub.publish(model_state_msg)
        wait_traj.publish(Bool(data=False))


def collisionCallBack(collision_msg):
    global trying_to_stabilize, w
    # if we have any collisions at all, discard those weights
    if collision_msg.states and not trying_to_stabilize:
        trying_to_stabilize = True
        # w += 1
        print("Writing to bad file")
        with open(bad_weights_file, 'a') as f:
            f.write("{},{},{}\n".format(a, b, c))


def cmdCallBack(cmd_msg):
    global v1, v2
    v1 = cmd_msg.linear.x
    v2 = cmd_msg.angular.z


def laserScanMsgCallBack(laser_msg):
    global direction_vector
    scan = laser_msg.ranges

    for counter, angle in enumerate(angles):
        if np.isinf(scan[angle]):
            direction_vector[counter] = 1 / laser_msg.range_max
        else:
            direction_vector[counter] = 1 / np.median([scan[angle - 2], scan[angle - 1],
                                                       scan[angle], scan[angle + 1],
                                                       scan[angle + 2]])


def set_wheel_velocities(left_wheel_speed=0.0, right_wheel_speed=0.0):
    global V_bias
    cmd_vel = Twist()

    # set linear forward velocity
    cmd_vel.linear.x = (right_wheel_speed + left_wheel_speed) / 2.0
    if cmd_vel.linear.x > MAX_TB3_LINEAR:
        V_bias = [v - 0.05 for v in V_bias]
    elif cmd_vel.linear.x < 0.1:
        V_bias = [v + 0.05 for v in V_bias]

    # set angular velocity
    cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) / (2 * half_wheel_separation)
    if cmd_vel.angular.z > 0:
        cmd_vel.angular.z = min(MAX_TB3_ANGULAR, cmd_vel.angular.z)
    else:
        cmd_vel.angular.z = max(-MAX_TB3_ANGULAR, cmd_vel.angular.z)

    cmd_vel_pub.publish(cmd_vel)
    # print(cmd_vel.linear.x, cmd_vel.angular.z, V_bias, w)


""""*******************************************************************************
* Control Loop function
*******************************************************************************"""


def controlLoop():
    # Velocities of L and R wheels are cross-connected to RHS and LHS sensors
    # left_vel = np.dot(WEIGHTS[0], direction_vector) + V_bias[0]
    # right_vel = np.dot(WEIGHTS[1], direction_vector) + V_bias[1]
    left_vel = np.dot(WEIGHTS[1], direction_vector) + V_bias[1]
    right_vel = np.dot(WEIGHTS[0], direction_vector) + V_bias[0]
    set_wheel_velocities(left_wheel_speed=left_vel.item(), right_wheel_speed=right_vel.item())
    wait_traj.publish(Bool(data=True))
    print(a, b, c, w)


"""*******************************************************************************
* Main function
*******************************************************************************"""

if __name__ == "__main__":
    rospy.init_node('Neural_net_turtlebot3')
    rospy.loginfo("To stop TurtleBot CTRL + C")

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmdCallBack)
    shut_trajectory = rospy.Publisher('/paramSearch', Bool, queue_size=3)
    wait_traj = rospy.Publisher('/can_log', Bool, queue_size=3)
    model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)
    laser_scan_sub = rospy.Subscriber('/scan', LaserScan, laserScanMsgCallBack)
    laps_sub = rospy.Subscriber('/laps', Int32, lapMsgCallBack)
    collision_sub = rospy.Subscriber('/robot_bumper', ContactsState, collisionCallBack)

    r = rospy.Rate(125)
    while not rospy.is_shutdown():
        if trying_to_stabilize:
            stabilize()
            trying_to_stabilize = False

        else:
            controlLoop()
            r.sleep()
