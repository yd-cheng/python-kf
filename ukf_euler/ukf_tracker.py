#!/usr/bin/env python2

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

from ukf import UKFTracker

class ROSTracker:
    
    def __init__(self, name, robot_ids):
        self.robot_ids = robot_ids
        self.ros_topics = ['robot_pose_{}'.format(id) for id in self.robot_ids]

        rospy.init_node(name)
        rospy.loginfo('Created ros node {}'.format(name))

        for id in self.robot_ids:
            rospy.loginfo('Publishing to tracked_robot_{}'.format(id))

        #init dict to store all publishers
        self.publishers = [rospy.Publisher('tracked_robot_{}'.format(id), Odometry, queue_size=10) for id in self.robot_ids]
        self.tracked_robots = [UKFTracker() for _ in self.robot_ids]

        self.rate = rospy.Rate(2)

    def start(self):
        for topic in self.ros_topics:
            rospy.Subscriber(topic, PoseStamped, self.callback)
        try:
            rospy.spin()
        except SystemExit:
            print("Tracker failed. Exiting")

    def callback(self, poseStamped):
        robot_id = int(poseStamped.header.frame_id)
        position = poseStamped.pose.position
        orientation = poseStamped.pose.orientation
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = rotation.as_euler('xyz', degrees=False)
        
        measurement = np.array([position.x, position.y, euler[2]])

        # Update and predict
        self.tracked_robots[robot_id].predict()
        self.tracked_robots[robot_id].update(measurement)

        # Publish tracking information
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()

        pose = self.tracked_robots[robot_id].getPose() # 3 element vector
        body_vel = self.tracked_robots[robot_id].getBodyVelocity() # 2 element vector
        global_vel = self.tracked_robots[robot_id].getGlobalVelocity() # 3 element vector

        rotation = R.from_euler('xyz', [pose[0], pose[1], pose[2]])
         
        quat = rotation.as_quat()

        odom.pose.pose = Pose(Point(pose[0], pose[1], 0), Quaternion(*quat))
        # Publishing global vel for testing
        odom.twist.twist = Twist(Vector3(global_vel[0], global_vel[1], 0), Vector3(0, 0, global_vel[2]))
    
        self.publishers[robot_id].publish(odom)

