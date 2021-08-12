#!/usr/bin/env python2

import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints

class UKFParams:
    
    # Sigma distribution params
    DIMENSIONS = 4
    ALPHA = 0.001
    BETA = 2.0
    KAPPA = 1.0

    # Covariance params
    INITIAL_STATE_COV = 1000

    # PROC=PROCESS;VEL=VELOCITY;ANG=ANGULAR; POS=POSITION
    POS_PROC_COV = 1
    VEL_PROC_COV = 10
    HEADING_PROC_COV = 0.05
    ANG_VEL_PROC_COV = 0.5

    # OBS=OBSERVATION
    POS_OBS_COV = 2
    HEADING_OBS_COV = 0.05

    def getMerwePoints():
        return MerweScaledSigmaPoints(DIMENSIONS, ALPHA, BETA, KAPPA)

    def getJulierPoints():
        #TODO
        pass

    def getXimplexPoints():
        #TODO
        pass

    def hfunction(state):
        x = state[0]
        y = state[2]
        heading = state[4]
        return np.array([x, y, heading]) # [x, y, heading]
       
    def state_transition(state, dt):
        heading = state[4]

        x_vel = state[1]
        y_vel = [3]
        x_body_vel = x_vel*numpy.cos(heading) - y_vel*numpy.sin(heading)
        y_body_vel = x_vel*numpy.sin(heading) + y_vel*numpy.cos(heading)

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

