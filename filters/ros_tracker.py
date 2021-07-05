#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Pose

from constants import TrackerConst
from robot_kalman_filter import RobotFilter


class ROSTracker:
    STATE_PREDICTION_TIME = TrackerConst.STATE_PREDICTION_TIME

    def __init__(self, name, ros_topic):
        self.ros_topic = ros_topic

        rospy.init_node(name)
        rospy.loginfo('Created ros node {}'.format(name))
        rospy.loginfo('Subscribed to {}'.format(ros_topic))

        # init dict to store all publishers
        self.pub_dict = {}
        self.robots = [RobotFilter() for _ in range(TrackerConst.MAX_ROBOT_PER_TEAM)]


    def start(self):
        # attempt to subscribe to given topic
        rospy.Subscriber(self.ros_topic, Pose, self.callback)
        try:
            rospy.spin()
        except SystemExit:
            print("Hello")



    def callback(self, pose):
        rospy.loginfo(pose.position)


