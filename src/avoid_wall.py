#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range


def laser_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)


def sonar_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.range)


def listeners():
    rospy.Subscriber('/robot0/laser_0', LaserScan, laser_callback)
    rospy.Subscriber('/robot0/sonar_0', Range, sonar_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('avoid_wall', anonymous=True)
    listeners()

