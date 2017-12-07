#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
PI = 3.1415926535897
theta = 0


def rotate(angle, velocity_publisher, clockwise=False, angular_speed=0.25):
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
    print("\n\nRotated successfully")
    print("Rotation was by {},  pi/2 is {}".format(abs(theta % (2 * PI) - initial_orientation % (2 * PI)), PI / 2.0))
    print("Error is ", (2 * abs(theta % (2 * PI) - initial_orientation % (2 * PI)) - PI) / PI)
    print("New orientation is: ", theta)


def go_forward(velocity_publisher):
    move_cmd = Twist()
    move_cmd.linear.x = 0.5
    r = rospy.Rate(5)
    # go forward 1 m (10 * (1/5Hz) * 0.5 m / seconds)
    rospy.loginfo("Going Straight")
    for _ in range(0, 10):
        velocity_publisher.publish(move_cmd)
        r.sleep()
    move_cmd.linear.x = 0.0
    velocity_publisher.publish(move_cmd)


def update_angle(geom_msg):
    global theta
    quaternion = geom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([quaternion.x,
                                                  quaternion.y,
                                                  quaternion.z,
                                                  quaternion.w])


def go_in_squares():
    rospy.init_node('walk_robot')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, update_angle)
    while not rospy.is_shutdown():
        go_forward(pub)
        rospy.sleep(1)
        rotate(90, pub)
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        go_in_squares()
    except rospy.ROSInterruptException:
        pass
