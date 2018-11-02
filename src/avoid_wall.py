#!/usr/bin/env python

import rospy
from math import cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range


def laser_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    linear = 0
    rotational = 0
    for i, laser_distance in enumerate(data.ranges):
        linear -= cos(data.angle_min + i * data.angle_increment) / (1.0 + laser_distance**2)
        rotational -= sin(data.angle_min + i * data.angle_increment) / (1.0 + laser_distance**2)

    linear /= len(data.ranges);
    rotational /= len(data.ranges);

    if (linear > 0.3):
        linear = 0.3;
    elif (linear < -0.3):
        linear = -0.3;

    vel_pub = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0.3 + linear
    vel_msg.angular.z = rotational
    vel_pub.publish(vel_msg)


def sonar_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.range)


def listeners():
    rospy.Subscriber('/robot0/laser_0', LaserScan, laser_callback)
    rospy.Subscriber('/robot0/sonar_0', Range, sonar_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('avoid_wall', anonymous=True)
    listeners()

