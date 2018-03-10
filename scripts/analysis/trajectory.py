#!/usr/bin/env python

import os
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool


(x_sim, y_sim) = 0, 0
(x_sim_prev, y_sim_prev) = 0, 0
LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")
at_cross_line = False

search_exhausted = False


def odomMsgCallBack(odom_msg):
    global x_sim, y_sim
    x_sim = odom_msg.pose.pose.position.x
    y_sim = odom_msg.pose.pose.position.y


def shutCallBack(shut_msg):
    if shut_msg.data == True:
        rospy.signal_shutdown("Simulation is finished")


def simple_track():
    global x_sim_prev, y_sim_prev, number_of_laps, at_cross_line

    i = int(input("Which log file to write to: -1 for automatic detection / any other integer for your choice\n"))
    filename = LOG_PATH + "/logs/trajectory_log_"
    if i < 0:
        i = 1
        while os.path.exists('{}{:d}.txt'.format(filename, i)):
            i += 1
    filename = '{}{:d}.txt'.format(filename, i)

    print("Writing a log file of trajectories to {}".format(filename))

    lap_reload = 5
    try:
        with open(filename, 'a') as tlog:
            while 1:
                # if it's not the first lap and we have moved at least a little in X or Y direction, then log it
                if number_of_laps > 0 and (round(x_sim_prev, 5) != round(x_sim, 5) or round(y_sim_prev, 5) != round(y_sim, 5)):
                    tlog.write('{:.6f},{:.6f}\n'.format(x_sim, y_sim))
                    x_sim_prev = x_sim
                    y_sim_prev = y_sim

                # if we are at the line where we started, then increase the lap number
                if abs(round(x_sim, 3) - start_x) < 0.01 and start_y-1 < round(y_sim, 3) < start_y+1:
                    if not at_cross_line:
                        at_cross_line = True
                        number_of_laps = (number_of_laps + 1) % lap_reload
                        print("New lap. Number {} started".format(number_of_laps))
                        laps = Int32(data=number_of_laps)
                        pub3.publish(laps)
                        r.sleep()
                else:
                    at_cross_line = False
                r.sleep()
    except Exception as e:
        print(e)


def param_search_track():
    global x_sim_prev, y_sim_prev, current_lap, at_cross_line

    i = int(input("Which log file to write to: -1 for automatic detection / any other integer for your choice\n"))
    file_name_ = LOG_PATH + "/logs/trajectory_log_"
    if i < 0:
        i = 1
        while os.path.exists('{}{:d}.txt'.format(file_name_, i)):
            i += 1

    LAP_RELOAD = 3

    try:
        while not rospy.is_shutdown():
            filename = '{}{:d}.txt'.format(file_name_, i)
            print("Writing a log file of trajectories to {}".format(filename))
            current_lap = -1
            with open(filename, 'a') as tlog:
                while current_lap < LAP_RELOAD:
                    # if it's not the first lap and we have moved at least a little in X or Y direction, then log it
                    if current_lap > 0 and (round(x_sim_prev, 5) != round(x_sim, 5) or round(y_sim_prev, 5) != round(y_sim, 5)):
                        tlog.write('{:.6f},{:.6f}\n'.format(x_sim, y_sim))
                        x_sim_prev = x_sim
                        y_sim_prev = y_sim

                    # if we are at the line where we started, then increase the lap number
                    if abs(round(x_sim, 3) - start_x) < 0.01 and start_y-1 < round(y_sim, 3) < start_y+1:
                        if not at_cross_line:
                            at_cross_line = True
                            current_lap = current_lap + 1
                            print("New lap. Number {} started".format(current_lap))
                            r.sleep()
                    else:
                        at_cross_line = False

                    laps = Int32(data=current_lap)
                    pub3.publish(laps)
                    r.sleep()
            i += 1
    except Exception as e:
        print(e)


if __name__ == "__main__":
    rospy.init_node('turtlebot3_trajectory')
    odometry_sub = rospy.Subscriber('/odom', Odometry, odomMsgCallBack)
    traj_shut_sub = rospy.Subscriber('/paramSearch', Bool, shutCallBack)
    # pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)
    # pub2 = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub3 = rospy.Publisher('/laps', Int32, queue_size=5)
    # reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

    r = rospy.Rate(50)
    r.sleep()
    if (x_sim, y_sim) != (0, 0):
        start_x = round(x_sim, 2)
        start_y = round(y_sim, 2)
        print("Start coordinates are {:.3f},{:.3f}".format(start_x, start_y))

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

    current_lap = -1
    param_search_track()
