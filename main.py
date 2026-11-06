import cv2
import numpy as np
from roboid import *
from utils.camera_supervisor import update_poses
from utils.formation_control import p_star, d_star
from utils.actuation import apply_control
import settings
from dataclasses import dataclass


@dataclass
class RobotAgent:
    marker_id: int
    robot_i: int
    hamster: HamsterS
    slot: int
    position: np.ndarray | None = None
    orientation: float | None = None


ROBOTS = {
    marker_id: RobotAgent(
        marker_id=marker_id,
        robot_i=i,
        hamster=HamsterS(i),
        slot=i,
    )
    for i, marker_id in enumerate(settings.target_ids)
}


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30)


try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        update_poses(ROBOTS, frame)

        if any(robot.position is None for robot in ROBOTS.values()):
            for robot in ROBOTS.values():
                robot.hamster.wheels(0, 0)

            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            continue

        positions = np.zeros((settings.ROBOTS_N, 2))

        for robot in ROBOTS.values():
            positions[robot.robot_i] = robot.position

        desired_positions = p_star()
        desired_offset = d_star(ROBOTS, desired_positions)

        #Create new axes reshape to reshape the same data to create pairwaise combinations
        actual_offset = positions[None, :, :] - positions[:, None, :]
        
        displacement_error = actual_offset - desired_offset

        u = np.sum(displacement_error, axis=1)

        for robot in ROBOTS.values():
            apply_control(
                robot,
                u[robot.robot_i],
                robot.orientation,
            )

        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    for robot in ROBOTS.values():
        robot.hamster.wheels(0, 0)

    cap.release()
    cv2.destroyAllWindows()
