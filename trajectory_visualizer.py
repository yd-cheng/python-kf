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
    #print('x: {}'.format(odom.pose.pose.position.x))
    #print('y: {}'.format(odom.pose.pose.position.y))
    dx = odom.twist.twist.linear.x
    dy = odom.twist.twist.linear.y

    if dx < 0:
        print('dx:{}'.format(odom.twist.twist.linear.x))
    else:
        print('dx: {}'.format(odom.twist.twist.linear.x))

    if dy < 0:
        print('dy:{}'.format(odom.twist.twist.linear.y))
    else:
        print('dy: {}'.format(odom.twist.twist.linear.y))


if __name__ == '__main__':
    initialize()
