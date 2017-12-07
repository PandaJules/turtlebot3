#!/usr/bin/env python

import rospy
from random import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
PI = 3.1416
wall_detected = False
forward, left, right, back = [0, 0, 0, 0]
front = 0.0
scan = []
theta = 0


def go_forward(vel_publisher):
    vel_msg = Twist()
    vel_msg.linear.x = 0.8
    print("Going at (linear, angular)", vel_msg.linear.x, vel_msg.angular.z)
    vel_publisher.publish(vel_msg)
    r.sleep()


def avoid(vel_publisher):
    print("\n=============\nEntered avoid loop, turning away")
    while 0.0 < front < 0.7:
        velocity_msg = Twist()
        velocity_msg.linear.x = 0
        vel_publisher.publish(velocity_msg)
        if sum(scan[0:90]) < sum(scan[-90:0]):
            rotate_with_odometry(angle=10, velocity_publisher=vel_publisher, clockwise=True)
        else:
            rotate_with_odometry(angle=10, velocity_publisher=vel_publisher, clockwise=False)
    print("Leaving avoid function\n===============")


def update_angle(geom_msg):
    global theta
    quaternion = geom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([quaternion.x,
                                                  quaternion.y,
                                                  quaternion.z,
                                                  quaternion.w])


def update_laser_scan(scan_msg):
    global front, scan, forward, left, right, wall_detected
    scan = scan_msg.ranges
    left = scan_msg.ranges[89]
    forward = scan_msg.ranges[0]
    front = min(scan_msg.ranges[-10:]+scan_msg.ranges[:10])
    right = scan_msg.ranges[269]
    # back = scan_msg.ranges[179]
    if 0 < front < 0.6:
        wall_detected = True
    else:
        wall_detected = False


def rotate_with_time(angle, velocity_publisher, clockwise=False, angular_speed=1):
    initial_orientation = theta
    relative_angle = angle * PI / 180

    vel_msg = Twist()
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while abs(current_angle - relative_angle) > 0.001:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print("\nRotated successfully")
    print("How close we got: {}".format(current_angle - relative_angle))
    print("Rotation was by {},  pi/2 is {}".format(abs(theta % (2 * PI) - initial_orientation % (2 * PI)), PI / 2.0))
    print("Error is ",
          (abs(theta % (2 * PI) - initial_orientation % (2 * PI)) % (2 * PI) - relative_angle) / relative_angle)
    print("New orientation is: {}\n".format(theta))


def rotate_with_odometry(angle, velocity_publisher, clockwise=False, angular_speed=0.25):
    initial_orientation = theta
    relative_angle = angle * PI / 180
    goal_angle = relative_angle + initial_orientation

    vel_msg = Twist()
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    while abs(theta % (2*PI) - goal_angle % (2*PI)) > 0.005:
        velocity_publisher.publish(vel_msg)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print("\nRotated successfully")
    print("Rotation was by {},  pi/2 is {}".format(abs(theta % (2 * PI) - initial_orientation % (2 * PI)), PI / 2.0))
    print("Error is ",
          (abs(theta % (2 * PI) - initial_orientation % (2 * PI)) % (2 * PI) - relative_angle) / relative_angle)
    print("New orientation is: {}\n".format(theta))


if __name__ == "__main__":

    rospy.init_node('walk_robot')
    rospy.loginfo("To stop TurtleBot CTRL + C")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    sub1 = rospy.Subscriber('/scan', LaserScan, update_laser_scan)
    sub2 = rospy.Subscriber('/odom', Odometry, update_angle)

    r = rospy.Rate(5)

    while not rospy.is_shutdown():
        if wall_detected:
            avoid(pub)
        else:
            go_forward(pub)
