#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry

(x_sim, y_sim) = 0, 0
LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")
inside_zone = False


def plot(path_to_coordinates, num):
    xs = []
    ys = []

    with open(path_to_coordinates, "r") as mycoords:
        mycoords.readline()
        for line in mycoords:
            [x, y] = map(float, line.split(","))
            xs.append(x)
            ys.append(y)

    plt.figure()
    plt.plot(xs, ys, 'b:')
    traj_name = LOG_PATH + "/screenshots" + "/traj"
    plt.savefig('{}{:d}.png'.format(traj_name, num))


def odomMsgCallBack(odom_msg):
    global x_sim, y_sim
    x_sim = odom_msg.pose.pose.position.x
    y_sim = odom_msg.pose.pose.position.y


def track():
    global inside_zone
    filename = LOG_PATH + "/logs/trajectory_log_"
    i = 1
    while os.path.exists('{}{:d}.txt'.format(filename, i)):
        i += 1
    filename = '{}{:d}.txt'.format(filename, i)

    r = rospy.Rate(10)
    try:
        tlog = open(filename, 'a')
        while 1:
            tlog.write('{:.6f},{:.6f}\n'.format(x_sim, y_sim))
            r.sleep()
            if 1.45 < x_sim < 1.6 and 5 < y_sim < 6:
                if not inside_zone:
                    inside_zone = True
                    print("new file", i)
                    tlog.close()
                    break
            else:
                inside_zone = False
        track()
    except Exception as e:
        print e


if __name__ == "__main__":
    rospy.init_node('turtlebot3_trajectory')
    odometry_sub = rospy.Subscriber('/odom', Odometry, odomMsgCallBack)
    track()
