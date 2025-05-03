import cv2
import numpy as np
import cv2.aruco as aruco

# --- CONFIG ---
calib_path = r"D:\masters\Personal Projects\Advance Mechatronics\Final Project\Aruco Markers\calibration_data.npz"
marker_length = 7.0  # in cm
ordered_ids = [0, 1, 2, 3]
robot_ids = [4, 5]
url = "http://10.18.238.136:4747/video"
live_feed_flag = False
image_path = r"C:\Users\Acer\Downloads\image_1_calib.png"

# --- Load calibration ---
calib = np.load(calib_path)
camera_matrix = calib["camera_matrix"]
dist_coeffs = calib["dist_coeffs"]

# --- ArUco setup ---
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_7X7_250)
params = aruco.DetectorParameters()

# --- Initialize capture or load image ---
if live_feed_flag:
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print("❌ Could not open video stream.")
        exit()
else:
    img = cv2.imread(image_path)
    if img is None:
        print("❌ Image not found.")
        exit()

def draw_rectangle_from_markers(img, tvecs_dict, camera_matrix, dist_coeffs, ordered_ids=[0, 1, 2, 3]):
    rect_points = []
    for mid in ordered_ids:
        pt = tvecs_dict[mid]
        imgpt, _ = cv2.projectPoints(np.array([pt]), np.zeros(3), np.zeros(3), camera_matrix, dist_coeffs)
        x, y = int(imgpt[0][0][0]), int(imgpt[0][0][1])
        rect_points.append((x, y))
        cv2.circle(img, (x, y), 6, (0, 255, 0), -1)
        cv2.putText(img, f"ID {mid}", (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    for i in range(4):
        cv2.line(img, rect_points[i], rect_points[(i + 1) % 4], (255, 0, 0), 2)

def compute_relative_pose(robot_tvec, origin, R_world):
    relative = robot_tvec - origin
    transformed = np.linalg.inv(R_world) @ relative
    return transformed

def get_theta_deg(rvec):
    R, _ = cv2.Rodrigues(rvec)
    angle = np.arctan2(R[1, 0], R[0, 0])  # yaw from rotation matrix
    return np.degrees(angle)

while True:
    if live_feed_flag:
        ret, img = cap.read()
        if not ret:
            break

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None:
        print("No markers detected.")
        if not live_feed_flag:
            break
        continue

    ids = ids.flatten()
    tvecs_dict = {}
    rvecs_dict = {}
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

    for i, marker_id in enumerate(ids):
        tvecs_dict[marker_id] = tvecs[i][0]
        rvecs_dict[marker_id] = rvecs[i][0]
        aruco.drawDetectedMarkers(img, [corners[i]], borderColor=(0,255,0))

    # Draw rectangle
    if all(mid in tvecs_dict for mid in ordered_ids):
        draw_rectangle_from_markers(img, tvecs_dict, camera_matrix, dist_coeffs)

        # Define new reference frame using ID 3 as origin
        origin = tvecs_dict[3]
        x_axis = tvecs_dict[2] - origin
        y_axis = tvecs_dict[0] - origin
        x_unit = x_axis / np.linalg.norm(x_axis)
        y_unit = y_axis / np.linalg.norm(y_axis)
        z_unit = np.cross(x_unit, y_unit)
        R_world = np.column_stack([x_unit, y_unit, z_unit])

        for rid in robot_ids:
            if rid in tvecs_dict:
                rel_pos = compute_relative_pose(tvecs_dict[rid], origin, R_world)
                x_cm, y_cm = rel_pos[0], rel_pos[1]
                theta = get_theta_deg(rvecs_dict[rid])

                imgpt, _ = cv2.projectPoints(np.array([tvecs_dict[rid]]), np.zeros(3), np.zeros(3), camera_matrix, dist_coeffs)
                cx, cy = int(imgpt[0][0][0]), int(imgpt[0][0][1])

                cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(img, f"ID:{rid}", (cx, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                cv2.putText(img, f"x={x_cm:.2f}cm y={y_cm:.2f}cm", (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
                cv2.putText(img, f"theta={theta:.1f}", (cx, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)

    # Resize for display
    display = cv2.resize(img, (0, 0), fx=0.6, fy=0.6)
    cv2.imshow("ArUco Tracker", display)
    key = cv2.waitKey(0)
    if key == 27:
        break

if live_feed_flag:
    cap.release()
cv2.destroyAllWindows()
