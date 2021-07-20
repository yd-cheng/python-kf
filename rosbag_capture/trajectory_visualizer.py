#!/usr/bin/env python2

import sys 
import rospy 
import numpy as np 
import tf
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry


def initialize():
    if len(sys.argv) < 2:
        print("Enter robot ID to plot")
        print("Exiting")
        sys.exit(0)
    
    robot_id = sys.argv[1]
    node_name = 'trajectory_{}_plot'.format(robot_id)
    topic_name = 'tracked_robot_{}'.format(robot_id)

    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber(topic_name, Odometry, callback)
    rospy.loginfo('Subscribed to tracked_robot_{}'.format(robot_id))

    rospy.spin()

def callback(odom):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    dx = odom.twist.twist.linear.x
    dy = odom.twist.twist.linear.y

    flag = False

    if flag == True:
        if dx < 0:
            print('dx:{}'.format(dx))
        else:
            print('dx: {}'.format(dx))

        if dy < 0:
            print('dy:{}'.format(dy))
        else:
            print('dy: {}'.format(dy))
    else:
        if x < 0:
            print('x:{}'.format(x))
        else:
            print('x: {}'.format(x))

        if y < 0:
            print('y:{}'.format(y))
        else:
            print('y: {}'.format(y))



if __name__ == '__main__':
    initialize()
