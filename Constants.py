NUM_UAVS = 3 #number of uavs
NUM_TARGETS = 3 #number of targets

UAV_LAYDOWN = [[-10,-10],[-10,0],[-10,5]]
TARGET_LAYDOWN = [[10, -10],[-10, 10],[2,2]]

UAV_SPEED = 2
TARGET_SPEED = 1.5

MAX_TEST_TIME = 100 #time after which the test should terminate
SIM_DT = 0.5
SIM_SIDE_LEN = 20 #simulator extends from (-SIM_SIDE_LEN, -SIM_SIDE_LEN) to (SIM_SIDE_LEN, SIM_SIDE_LEN)

COMM_DISTANCE = 5 #max distance away targets can share world models
UAV_DETECTION_DIST = 5 #max distance away uavs can detect other uavs without sharing world models
TARGET_DETECTION_DIST = 5 #max distance away uavs can see targets

#For demo test
#uav attraction: -1.0 + -1.0*i^4 + 0.8*d^1*i^3 + -1.6*d^2*i^2 + 0.6*d^3*i^1 + 1.4*d^4 + 1.2*d^5*i^-1
#target attraction: -0.0 + 1.2*i^4 + 0.4*d^1*i^3 + -1.4*d^2*i^2 + 0.6*d^3*i^1 + -0.2*d^4 + 0.4*d^5*i^-1
