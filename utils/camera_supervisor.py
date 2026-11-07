
import math
import numpy as np
import cv2
import settings

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)


def update_poses(ROBOTS, frame):
    # Reset all poses before processing the current frame
    for robot in ROBOTS.values():
        robot.position = None
        robot.orientation = None

    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return

    ids = ids.flatten()

    for marker_corner, marker_id in zip(corners, ids):
        marker_id = int(marker_id)

        # Ignore markers not associated with our robots
        if marker_id not in ROBOTS:
            continue

        marker_corner = marker_corner.reshape((4, 2))

        center = np.mean(marker_corner, axis=0)
        topLeft, topRight, bottomRight, bottomLeft = marker_corner
        heading_vector = ((topLeft+topRight)-(bottomLeft+bottomRight))/2

        orientation = math.degrees(
            math.atan2(
                heading_vector[1],
                heading_vector[0]
            )
        )

        # For negative angles
        orientation %= 360

        ROBOTS[marker_id].position = center
        ROBOTS[marker_id].orientation = orientation
