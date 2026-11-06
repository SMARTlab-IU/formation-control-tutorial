from roboid import *


#Pixel to CM conversion for 
PX_TO_CM = 0.0374

target_ids = [15, 16, 17, 90]

ROBOTS_N = 4
SENSING_R = 500
SHAPE_R = 100

#Controller Gains
KP_DISTANCE = .7
KP_ANGLE = .5

#Acceptable Error Thresholds
DISTANCE_THRESHOLD =  10
ANGLE_THRESHOLD = 30

OBS_THRESHOLD = 70
OBS_SPEED = 50
