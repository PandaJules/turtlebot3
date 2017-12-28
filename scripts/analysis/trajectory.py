#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry

(x_sim, y_sim) = 0, 0
LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")


def plot(path_to_coordinates):
    xs = []
    ys = []

    with open(path_to_coordinates, "r") as mycoords:
        for line in mycoords:
            [x, y] = map(float, line.split(","))
            xs.append(x)
            ys.append(y)

    plt.figure()
    plt.plot(xs, ys)
    traj_name = "/home/julia/Desktop/traj"
    j = 0
    while os.path.exists('{}{:d}.png'.format(traj_name, j)):
        j += 1
    plt.savefig('{}{:d}.png'.format(traj_name, j))


def odomMsgCallBack(odom_msg):
    global x_sim, y_sim
    x_sim = odom_msg.pose.pose.position.x
    y_sim = odom_msg.pose.pose.position.y


if __name__ == "__main__":

    rospy.init_node('turtlebot3_trajectory')
    odometry_sub = rospy.Subscriber('/odom', Odometry, odomMsgCallBack)

    r = rospy.Rate(25)

    filename = LOG_PATH + "/trajectory_log"
    i = 0
    while os.path.exists('{}{:d}.txt'.format(filename, i)):
        i += 1
    filename = '{}{:d}.txt'.format(filename, i)
    
    try:
        with open(filename, 'a') as tlog:
            while 1:
                tlog.write('{:.5f},{:.5f}\n'.format(x_sim, y_sim))
                r.sleep()
    except Exception as e:
        print e
    finally:
        plot(filename)
