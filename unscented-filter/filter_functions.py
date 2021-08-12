#!/usr/bin/env python3

import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints

class UKFParams:
    
    # Sigma distribution params
    STATE_DIM = 8
    ALPHA = 0.001
    BETA = 2.0
    KAPPA = 1.0

    INITIAL_STATE_COV = 1000
    # Process covariance constants
    POSITION_PROCESS_COV = 1
    VELOCITY_PROCESS_COV = 10
    HEADING_PROCESS_COV = 0.05
    ANGULAR_VEL_PROCESS_COV = 0.5
    # Measurement covariance
    POSITION_OBSERVATION_COV = 2
    HEADING_OBSERVATION_COV = 0.05


    def getMerwePoints():
        return MerweScaledSigmaPoints(DIMENSIONS, ALPHA, BETA, KAPPA)

    def getJulierPoints():
        #TODO
        raise NotImplementedError

    def getSimplexPoints():
        #TODO
        raise NotImplementedError

    def getProcessCov():
        return np.diag([POSITION_PROCESS_COV,
                       VELOCITY_PROCESS_COV,
                       POSITION_PROCESS_COV,
                       VELOCITY_PROCESS_COV,
                       HEADING_PROCESS_COV,
                       ANGULAR_VEL_PROCESS_COV,
                       VELOCITY_PROCESS_COV,
                       VELOCITY_PROCESS_COV])

    def getInitialStateCov():
        return INITIAL_STATE_COV * np.eye(STATE_DIM)

    def getInitialState():
        return np.zeros(STATE_DIM) 

    def hfunction(state):
        x = state[0]
        y = state[2]
        heading = state[4]
        return np.array([x, y, heading]) # [x, y, heading]
       
    def state_transition(state, dt):
        heading = state[4]

        x_vel = state[1]
        y_vel = [3]
        x_body_vel = x_vel*np.cos(heading) - y_vel*np.sin(heading)
        y_body_vel = x_vel*np.sin(heading) + y_vel*np.cos(heading)

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


