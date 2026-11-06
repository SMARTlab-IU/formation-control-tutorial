import math 
import numpy as np
import cv2
import settings

#cv2
#library for marker type (Use 4x4 if trouble detecting robots, simpler shapes)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
detector = cv2.aruco.ArucoDetector(aruco_dict)


def update_poses(ROBOTS, frame):
    detected_ids = set()
    #ids not detected in order
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return
    
    ids = ids.flatten()
        
    for marker_corner, marker_id in zip(corners, ids):
        marker_corner = marker_corner.reshape((4, 2))

        center = np.mean(marker_corner, axis=0)
        topLeft, topRight, bottomRight, bottomLeft = marker_corner
        heading_vector = ((topLeft+topRight)-(bottomLeft+bottomRight))/2

        orientation = math.degrees(
            math.atan2(
                heading_vector[1],
                heading_vector[0]
        ))

        #for negative angles
        orientation %= 360


        ROBOTS[marker_id].position = center
        ROBOTS[marker_id].orientation = orientation

        detected_ids.add(marker_id)

        # Any robot not detected this frame becomes missing
    for marker_id, robot in ROBOTS.items():
        if marker_id not in detected_ids:
            robot.position = None
            robot.orientation = None
                

        

           