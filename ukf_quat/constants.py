class UKFParams:
    # Sigma distribution params
    STATE_DIM = 10
    MEASURE_DIM = 4
    ALPHA = 0.001
    BETA = 2
    KAPPA = 1.0
    DELTA_T = 0.03

    INITIAL_STATE_COV = 1000.0
    # Process covariance constants
    POSITION_PROCESS_COV = 1.0
    VELOCITY_PROCESS_COV = 10.0
    HEADING_PROCESS_COV = 0.05
    ANGULAR_VEL_PROCESS_COV = 0.5
    # Measurement covariance
    POSITION_OBSERVATION_COV = 2.0
    HEADING_OBSERVATION_COV = 0.05
