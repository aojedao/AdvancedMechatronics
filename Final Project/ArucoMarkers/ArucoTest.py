#!/usr/bin/env python3

import numpy as np
import cv2
import cv2.aruco as aruco
import math

# Store initial positions for relative movement
initial_positions = {}

def detect_ArUco(img):
    Detected_ArUco_markers = {}
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters()

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        for i in range(len(ids)):
            Detected_ArUco_markers[ids[i][0]] = corners[i]
    return Detected_ArUco_markers   

def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ArUco_marker_angles = {}
    for key in Detected_ArUco_markers:
        corners = Detected_ArUco_markers[key]
        tl = corners[0][0]
        tr = corners[0][1]
        br = corners[0][2]
        bl = corners[0][3]

        top_mid = (tl[0] + tr[0]) / 2, -(tl[1] + tr[1]) / 2
        center = (tl[0] + tr[0] + br[0] + bl[0]) / 4, -(tl[1] + tr[1] + br[1] + bl[1]) / 4

        try:
            angle = math.degrees(np.arctan2((top_mid[1] - center[1]), (top_mid[0] - center[0])))
        except:
            angle = 0
        angle = (angle + 360) % 360
        ArUco_marker_angles.update({key: angle})
    return ArUco_marker_angles

def get_relative_xy_theta(Detected_ArUco_markers, ArUco_marker_angles):
    ArUco_marker_pose = {}
    for key in Detected_ArUco_markers:
        corners = Detected_ArUco_markers[key]
        tl = corners[0][0]
        tr = corners[0][1]
        br = corners[0][2]
        bl = corners[0][3]
        center = np.array([(tl[0] + tr[0] + br[0] + bl[0]) / 4.0, (tl[1] + tr[1] + br[1] + bl[1]) / 4.0])
        
        if key not in initial_positions:
            initial_positions[key] = center

        displacement = center - initial_positions[key]
        dx = displacement[0]
        dy = -displacement[1]  # Invert Y to match Cartesian "up is +Y"

        ArUco_marker_pose[key] = (round(dx, 2), round(dy, 2), round(ArUco_marker_angles[key], 2))
    return ArUco_marker_pose

def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles, ArUco_marker_pose):
    for key in Detected_ArUco_markers:
        corners = Detected_ArUco_markers[key]
        tl = corners[0][0]
        tr = corners[0][1]
        br = corners[0][2]
        bl = corners[0][3]
        center = int((tl[0]+tr[0]+br[0]+bl[0])//4), int((tl[1]+tr[1]+br[1]+bl[1])//4)
        top = int((tl[0]+tr[0])//2), int((tl[1]+tr[1])//2)
        
        # Draw visual markers
        img = cv2.line(img, top, center, (255, 0, 0), 3)
        img = cv2.circle(img, center, 6, (0, 0, 255), -1)
        img = cv2.putText(img, f"ID:{key}", (center[0]+10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        img = cv2.putText(img, f"theta:{ArUco_marker_angles[key]:.1f}", (center[0]+10, center[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        x, y, theta = ArUco_marker_pose[key]
        img = cv2.putText(img, f"x:{x}px y:{y}px", (center[0]+10, center[1]+40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
    return img

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Camera couldn't be opened")
        return

    while True:
        success, img = cap.read()
        if not success:
            continue

        Detected_ArUco_markers = detect_ArUco(img)
        if Detected_ArUco_markers:
            angles = Calculate_orientation_in_degree(Detected_ArUco_markers)
            poses = get_relative_xy_theta(Detected_ArUco_markers, angles)
            img = mark_ArUco(img, Detected_ArUco_markers, angles, poses)

        cv2.imshow("ArUco Tracker", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
