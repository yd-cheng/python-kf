#!/usr/bin/env python3
import sys
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

from constants import TrackerConst
from robot_kalman_filter import RobotFilter


class ROSTracker:
    STATE_PREDICTION_TIME = TrackerConst.STATE_PREDICTION_TIME

    def __init__(self, name, robot_ids):
        self.robot_ids = robot_ids
        self.ros_topics = ['robot_pose_{}'.format(id) for id in self.robot_ids]

        rospy.init_node(name)
        rospy.loginfo('Created ros node {}'.format(name))
        
        for topic in self.ros_topics:
            rospy.loginfo('Subscribed to {}'.format(topic))

        # init dict to store all publishers
        self.publisher = [rospy.Publisher('tracked_robot_{}'.format(id), Odometry, queue_size=10) for id in self.robots_ids]
        self.tracked_robots = [RobotFilter() for _ in self.robot_ids]

        # Sleep so CPU isn't hammered
        self.rate = rospy.Rate(2)


    def start(self):
        # attempt to subscribe to given topic
        rospy.Subscriber(self.ros_topic, PoseStamped, self.callback)
        try:
            rospy.spin()
        except SystemExit:
            print("Tracker failed. Exiting")



    def callback(self, poseStamped):
        time = poseStamped.header.stamp
        robot_id = poseStamped.header.frame_id
        print(robot_id)
        pose = poseStamped.pose
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        obs_state = np.array([pose.position.x, pose.position.y, euler[2]]) # euler[2] is orientation in radians

        # Update and predict
        self.tracked_robot.update(obs_state, time) 
        self.tracked_robot.predict(TrackerConst.STATE_PREDICTION_TIME)

        #rospy.loginfo('dx: %0.3f, dy: %0.3f', self.tracked_robot.velocity[0], self.tracked_robot.velocity[1])
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()

        robot_pose = self.tracked_robot.pose
        robot_velocity = self.tracked_robot.velocity

        quat = tf.transformations.quaternion_from_euler(0, 0, self.tracked_robot.get_orientation)

        odom.pose.pose = Pose(Point(robot_pose[0], robot_pose[1], 0), Quaternion(*quat))
        odom.twist.twist = Twist(Vector3(robot_velocity[0], robot_velocity[1], 0), Vector3(0, 0, robot_velocity[2]))
      
        self.publisher.publish(odom)
        #self.rate.sleep()
