
import settings
from roboid import *
import math 
import numpy as np


def apply_control(robot, u, theta):
   
    left = robot.hamster.left_proximity()
    right = robot.hamster.right_proximity()
    
    if left > settings.OBS_THRESHOLD and right > settings.OBS_THRESHOLD:
        robot.hamster.wheels(-settings.OBS_SPEED, -settings.OBS_SPEED // 2)
        return

    if left > settings.OBS_THRESHOLD:
        robot.hamster.wheels(settings.OBS_SPEED, -settings.OBS_SPEED)
        return
    
    if right > settings.OBS_THRESHOLD:
        robot.hamster.wheels(-settings.OBS_SPEED, settings.OBS_SPEED)
        return
   
    ux = u[0]
    uy = u[1]

    distance = np.linalg.norm(u)

    if distance <= settings.DISTANCE_THRESHOLD:
        robot.hamster.wheels(0, 0)
        return

    target_angle = math.degrees(math.atan2(uy, ux))

    angle_error = target_angle - theta
    angle_error = (angle_error + 180) % 360 - 180
    
    if abs(angle_error) > settings.ANGLE_THRESHOLD:

        turn_speed = min(settings.KP_ANGLE * abs(angle_error), 40)

        if angle_error > 0:
            robot.hamster.wheels(turn_speed, -turn_speed)
        else:
            robot.hamster.wheels(-turn_speed, turn_speed)
    else:
        forward_speed = min(settings.KP_DISTANCE * distance, 40)
        robot.hamster.wheels(forward_speed, forward_speed)
    return(angle_error)
