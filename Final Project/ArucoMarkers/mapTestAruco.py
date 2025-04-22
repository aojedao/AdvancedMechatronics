# -----------------------------------------------------------------------------
# Imports
# -----------------------------------------------------------------------------
import cv2
import numpy as np
import time # Optional, for FPS calculation

# -----------------------------------------------------------------------------
# --- User Configuration - MUST EDIT THESE VALUES ---
# -----------------------------------------------------------------------------

# --- ArUco Dictionary ---
# Choose the dictionary that your markers belong to.
# Examples: cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_6X6_250, cv2.aruco.DICT_7X7_1000 etc.
ARUCO_DICT_NAME = cv2.aruco.DICT_6X6_250
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
aruco_params = cv2.aruco.DetectorParameters()

# --- Board Definition ---
# Define the properties of your physical board setup. Measure carefully!
# Units should be consistent (e.g., meters).

MARKER_SIZE = 0.035 # Side length of one ArUco marker in meters (e.g., 3.5 cm)
SQUARE_SIDE_LENGTH = 0.20 # Side length of the square formed by the *centers* of the markers (e.g., 20 cm)

# --- Marker IDs ---
# Assign the IDs of the markers you are using at the corners.
# IMPORTANT: The order corresponds to the obj_points order defined below.
#            Make sure this matches your physical layout.
# Example: Marker ID 10 is top-right, 11 is top-left, 12 is bottom-left, 13 is bottom-right
MARKER_ID_TR = 10 # Top-Right
MARKER_ID_TL = 11 # Top-Left
MARKER_ID_BL = 12 # Bottom-Left
MARKER_ID_BR = 13 # Bottom-Right
marker_ids_list = np.array([MARKER_ID_TR, MARKER_ID_TL, MARKER_ID_BL, MARKER_ID_BR])

# --- Board 3D Coordinates ---
# Define the 3D coordinates of the MARKER CORNERS in your map's coordinate system.
# The origin (0,0,0) is assumed to be the center of the square on the floor (Z=0).
# Order of corners for each marker: Top-Left, Top-Right, Bottom-Right, Bottom-Left

half_square = SQUARE_SIDE_LENGTH / 2.0
half_marker = MARKER_SIZE / 2.0

# Corners for Marker 1 (Top-Right Quadrant)
corners_m1 = np.array([
    [ half_square - half_marker,  half_square + half_marker, 0], # Top-Left
    [ half_square + half_marker,  half_square + half_marker, 0], # Top-Right
    [ half_square + half_marker,  half_square - half_marker, 0], # Bottom-Right
    [ half_square - half_marker,  half_square - half_marker, 0]  # Bottom-Left
], dtype=np.float32)

# Corners for Marker 2 (Top-Left Quadrant)
corners_m2 = np.array([
    [-half_square - half_marker,  half_square + half_marker, 0], # Top-Left
    [-half_square + half_marker,  half_square + half_marker, 0], # Top-Right
    [-half_square + half_marker,  half_square - half_marker, 0], # Bottom-Right
    [-half_square - half_marker,  half_square - half_marker, 0]  # Bottom-Left
], dtype=np.float32)

# Corners for Marker 3 (Bottom-Left Quadrant)
corners_m3 = np.array([
    [-half_square - half_marker, -half_square + half_marker, 0], # Top-Left
    [-half_square + half_marker, -half_square + half_marker, 0], # Top-Right
    [-half_square + half_marker, -half_square - half_marker, 0], # Bottom-Right
    [-half_square - half_marker, -half_square - half_marker, 0]  # Bottom-Left
], dtype=np.float32)

# Corners for Marker 4 (Bottom-Right Quadrant)
corners_m4 = np.array([
    [ half_square - half_marker, -half_square + half_marker, 0], # Top-Left
    [ half_square + half_marker, -half_square + half_marker, 0], # Top-Right
    [ half_square + half_marker, -half_square - half_marker, 0], # Bottom-Right
    [ half_square - half_marker, -half_square - half_marker, 0]  # Bottom-Left
], dtype=np.float32)

# Combine corner definitions - ORDER MUST MATCH marker_ids_list
obj_points = [corners_m1, corners_m2, corners_m3, corners_m4]

# Create the ArUco Board object
board = cv2.aruco.Board(obj_points, aruco_dict, marker_ids_list)

# --- Camera Calibration Parameters ---
# !! Replace these with your actual camera calibration results !!
#    (Obtained using cv2.calibrateCamera)
# Example values (likely incorrect for your camera)
camera_matrix = np.array([
    [1000, 0, 640], # fx, 0, cx
    [0, 1000, 360], # 0, fy, cy
    [0, 0, 1]
], dtype=np.float32)

distortion_coeffs = np.array([0.1, -0.05, 0, 0, 0], dtype=np.float32) # k1, k2, p1, p2, k3

# --- Camera Setup ---
CAMERA_INDEX = 0 # Index of the camera to use (e.g., 0 for default webcam)
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print(f"Error: Could not open camera {CAMERA_INDEX}")
    exit()

# Set camera resolution (optional, but recommended for consistency)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# -----------------------------------------------------------------------------
# --- Main Loop ---
# -----------------------------------------------------------------------------
print("Starting detection loop. Press 'q' to quit.")
axis_length = SQUARE_SIDE_LENGTH / 2.0 # Length of the coordinate axes to draw

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame")
        break

    # --- Marker Detection ---
    # Detect ArUco markers in the frame
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    # --- Board Pose Estimation ---
    rvec = None
    tvec = None
    if ids is not None and len(ids) > 0:
        # Draw detected markers for visualization
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate the pose of the entire board
        # retval is the number of markers used in the estimation
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, board, camera_matrix, distortion_coeffs, None, None
        )

        # If pose estimation was successful (found at least one marker from the board)
        if retval > 0:
            # Draw the coordinate axes of the board (your map origin)
            # The axes show the orientation of the board's coordinate system.
            # X = Red, Y = Green, Z = Blue
            cv2.drawFrameAxes(frame, camera_matrix, distortion_coeffs, rvec, tvec, axis_length)

            # --- Using the Pose ---
            # rvec: Rotation vector (axis-angle representation)
            # tvec: Translation vector
            # These vectors describe the transformation from the Board's coordinate
            # system (your map) to the Camera's coordinate system.
            #
            # Example: tvec = [tx, ty, tz] means the board's origin (center of the square)
            # is tx meters along the camera's X-axis, ty along Y, and tz along Z (distance).
            #
            # Example: You can convert rvec to a rotation matrix using cv2.Rodrigues(rvec)
            # if needed for further calculations.

            # print(f"Board Pose: Rvec={rvec.flatten()}, Tvec={tvec.flatten()}") # Uncomment to print pose

    # --- Display Output ---
    cv2.imshow("ArUco Board Detection", frame)

    # --- Exit Condition ---
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("Exiting...")
        break

# -----------------------------------------------------------------------------
# --- Cleanup ---
# -----------------------------------------------------------------------------
cap.release()
cv2.destroyAllWindows()
print("Resources released.")