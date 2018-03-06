#!/usr/bin/env python

import os
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState


(x_sim, y_sim) = 0, 0
(x_sim_prev, y_sim_prev) = 0, 0
LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")


def odomMsgCallBack(odom_msg):
    global x_sim, y_sim
    x_sim = odom_msg.pose.pose.position.x
    y_sim = odom_msg.pose.pose.position.y


def track():
    global x_sim_prev, y_sim_prev
    filename = LOG_PATH + "/logs/trajectory_log_"
    i = 1
    while os.path.exists('{}{:d}.txt'.format(filename, i)):
        i += 1
    filename = '{}{:d}.txt'.format(filename, i)

    r = rospy.Rate(10)
    try:
        with open(filename, 'a') as tlog:
            while 1:
                if round(x_sim_prev, 5) != round(x_sim, 5) or round(y_sim_prev, 5) != round(y_sim, 5):
                    tlog.write('{:.6f},{:.6f}\n'.format(x_sim, y_sim))
                    x_sim_prev = x_sim
                    y_sim_prev = y_sim
                r.sleep()
    except Exception as e:
        print e


if __name__ == "__main__":
    rospy.init_node('turtlebot3_trajectory')
    odometry_sub = rospy.Subscriber('/odom', Odometry, odomMsgCallBack)
    pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)
    pub2 = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

    # model_state_msg = ModelState()
    # model_state_msg.model_name = 'turtlebot3_burger'
    # model_state_msg.pose.orientation.x = 0
    # model_state_msg.pose.orientation.y = 0
    # model_state_msg.pose.orientation.z = 0
    # model_state_msg.pose.orientation.w = 1
    # model_state_msg.pose.position.x = 0
    # model_state_msg.pose.position.y = 2.5
    # model_state_msg.pose.position.z = 0
    # model_state_msg.twist.linear.x = 0
    # model_state_msg.twist.angular.z = 0

    track()
