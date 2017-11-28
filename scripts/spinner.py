#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
PI = 3.1415926535897
theta = 0


def rotate_with_time(angle, velocity_publisher, clockwise=False):
    initial_orientation = theta
    angular_speed = 1
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
    print "\n\nRotated successfully"
    print "How close we got: {}".format(current_angle-relative_angle)
    print "Rotation was by {},  pi/2 is {}".format(abs(theta%(2*PI) - initial_orientation%(2*PI)), PI/2.0)
    print "Error is ", (abs(theta%(2*PI) - initial_orientation%(2*PI))-relative_angle)/relative_angle
    print "New orientation is: ", theta


def rotate_with_odometry(angle, velocity_publisher, clockwise=False, angular_speed=0.25):
    initial_orientation = theta
    relative_angle = angle * PI / 180
    goal_angle = relative_angle + initial_orientation

    vel_msg = Twist()
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    while abs(theta%(2*PI) - goal_angle%(2*PI)) > 0.005:
        velocity_publisher.publish(vel_msg)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print "\n\nRotated successfully"
    print "Rotation was by {},  pi/2 is {}".format(abs(theta%(2*PI) - initial_orientation%(2*PI)), PI/2.0)
    print "Error is ", (abs(theta%(2*PI) - initial_orientation%(2*PI))-relative_angle)/relative_angle
    print "New orientation is: ", theta


def update_angle(geom_msg):
    global theta
    quaternion = geom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([quaternion.x,
                                                  quaternion.y,
                                                  quaternion.z,
                                                  quaternion.w])


def spin_on_the_spot():
    rospy.init_node('walk_robot')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    sub = rospy.Subscriber('/odom', Odometry, update_angle)
    while not rospy.is_shutdown():
        rotate_with_odometry(90, pub)
        rospy.sleep(3)


if __name__ == "__main__":
    try:
        spin_on_the_spot()
    except rospy.ROSInterruptException:
        pass

