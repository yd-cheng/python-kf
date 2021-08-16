#!/usr/bin/env python3

import sys
import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from constants import UKFParams

import rospy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
np.set_printoptions(linewidth=np.inf)

# Global objects
UKFTracker = None
robot_id = None
ros_topic = None
ros_publisher = None

def getMerwePoints():
    return MerweScaledSigmaPoints(UKFParams.STATE_DIM, UKFParams.ALPHA, UKFParams.BETA, UKFParams.KAPPA)

def getProcessCov():
    return np.diag([UKFParams.POSITION_PROCESS_COV,
                    UKFParams.VELOCITY_PROCESS_COV,
                    UKFParams.POSITION_PROCESS_COV,
                    UKFParams.VELOCITY_PROCESS_COV,
                    UKFParams.HEADING_PROCESS_COV,
                    UKFParams.ANGULAR_VEL_PROCESS_COV,
                    UKFParams.VELOCITY_PROCESS_COV,
                    UKFParams.VELOCITY_PROCESS_COV])

def getMeasurementCov():
    return np.diag([UKFParams.POSITION_OBSERVATION_COV,
                    UKFParams.POSITION_OBSERVATION_COV,
                    UKFParams.HEADING_OBSERVATION_COV])

def getInitialState():
    return np.random.randn(UKFParams.STATE_DIM) 

def getInitialStateCov():
    return UKFParams.INITIAL_STATE_COV * np.eye(UKFParams.STATE_DIM)

def getBodyVelocity():
    global UKFTracker
    state = UKFTracker.x_post
    return np.array([state[6], state[7]]).flatten()

def getGlobalVelocity():
    global UKFTracker
    state = UKFTracker.x_post
    return np.array([state[1], state[3], state[5]]).flatten()

def getPose():
    global UKFTracker
    state = UKFTracker.x_post
    return np.array([state[0], state[2], state[4]]).flatten()

def hfunction(state):
    x_pos = state[0]
    y_pos = state[2]
    heading = state[4]
    return np.array([x_pos, y_pos, heading])

def state_transition(state, dt):
    heading = state[4]

    x_vel = state[1]
    y_vel = state[3]
    x_body_vel = x_vel*np.cos(heading) - y_vel*np.sin(heading)
    y_body_vel = x_vel*np.sin(heading) + y_vel*np.cos(heading)
    print(x_body_vel)
    print(y_body_vel)

    F = np.array([[1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # x
                  [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # x_vel
                  [0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0], # y
                  [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], # y_vel
                  [0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0], # theta
                  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], # theta_vel
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, x_body_vel, 0.0], # x_body_vel
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, y_body_vel]  # y_body_vel
                  ], dtype=float)

    return np.dot(F, state)

def tracker_predict():
    global UKFTracker
    UKFTracker.predict()

def tracker_update(measurement):
    global UKFTracker
    UKFTracker.update(measurement)

def initialize_tracker():
    sigmas = getMerwePoints()
    global UKFTracker
    UKFTracker = UKF(dim_x=UKFParams.STATE_DIM, dim_z=UKFParams.MEASURE_DIM, fx=state_transition, hx=hfunction, dt=UKFParams.DELTA_T, points=sigmas)
    UKFTracker.x = getInitialState()
    UKFTracker.R = getMeasurementCov()
    UKFTracker.Q = getProcessCov()
    UKFTracker.P = getInitialStateCov()

    # Ros initializations
    global ros_topic
    ros_topic = 'tracked_robot_{}'.format(robot_id)
    rospy.init_node(ros_topic)
    rospy.loginfo('Publishing to tracked_robot_{}'.format(robot_id))

    global ros_publisher
    ros_publisher = rospy.Publisher(ros_topic, Odometry, queue_size=10)

def callback(poseStamped):
    robot_id = int(poseStamped.header.frame_id)
    position = poseStamped.pose.position
    orientation = poseStamped.pose.orientation
    rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    euler = rotation.as_euler('xyz', degrees=False)

    measurement = np.array([position.x, position.y, euler[2]])

    # Update and predict
    tracker_predict()
    tracker_update(measurement)

    # Publish tracking infomation
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    
    pose = getPose()
    body_vel = getBodyVelocity()
    global_vel = getGlobalVelocity()

    rotation = R.from_euler('xyz', [0.0, 0.0, pose[2]], degrees=False)
    quat = rotation.as_quat()

    odom.pose.pose = Pose(Point(pose[0], pose[1], 0), Quaternion(*quat))
    odom.twist.twist = Twist(Vector3(global_vel[0], global_vel[1], 0), Vector3(0, 0, global_vel[2]))
    ros_publisher.publish(odom)

def start():
    rospy.Subscriber('robot_pose_{}'.format(robot_id), PoseStamped, callback)
    try:
        rospy.spin()
    except SystemExit:
        print("Tracker failed. Exiting")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Enter id of robot')
        print('Exiting')
        sys.exit(1)

    robot_id = sys.argv[1]
    
    initialize_tracker()
    start()


