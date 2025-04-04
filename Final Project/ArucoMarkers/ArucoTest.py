import cv2
import cv2.aruco as aruco
import numpy as np

# ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Initialize the webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None and len(ids) >= 4:
        aruco.drawDetectedMarkers(frame, corners, ids)

        marker_corners = []
        for i in range(min(4, len(corners))):
            marker_corners.append(corners[i][0])

        marker_centers = []
        for corner in marker_corners:
            center_x = int(np.mean(corner[:, 0]))
            center_y = int(np.mean(corner[:, 1]))
            marker_centers.append((center_x, center_y))

        if len(marker_centers) == 4:
            points = np.array(marker_centers, dtype=np.int32)
            hull = cv2.convexHull(points)
            pts = hull.reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 3)

            origin = marker_centers[0]
            cv2.arrowedLine(frame, origin, (origin[0] + 50, origin[1]), (0, 0, 255), 2)
            cv2.arrowedLine(frame, origin, (origin[0], origin[1] - 50), (0, 0, 255), 2)
            print(f"Origin (First Marker): {origin}")

    cv2.imshow("Webcam with ArUco Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()