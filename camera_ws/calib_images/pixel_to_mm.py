import cv2
import numpy as np

# === CONFIGURATION ===
checkerboard_size = (15, 18)  # (columns, rows) of inner corners
square_size = 22.68  # size of one square in mm


# === LOAD CALIBRATION ===
with np.load("calibration_data.npz") as data:
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]


# === OBJECT POINTS ===
# 3D points in real world space (Z = 0)
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size


# === CLICK POINT HANDLER ===
clicked_point = None

def click_event(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)


# === CAMERA SETUP ===
cap = cv2.VideoCapture(0)
cv2.namedWindow("Pixel to mm")
cv2.setMouseCallback("Pixel to mm", click_event)


# === MAIN LOOP ===
while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, checkerboard_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if found:
        cv2.drawChessboardCorners(frame, checkerboard_size, corners, found)

        # === POSE ESTIMATION ===
        retval, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)

        if clicked_point:
            # Undistort the clicked pixel point
            pts = np.array([[[clicked_point[0], clicked_point[1]]]], dtype=np.float32)
            undistorted_pts = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=camera_matrix)

            # === 3D CONVERSION ===
            R, _ = cv2.Rodrigues(rvec)
            R_inv = np.linalg.inv(R)
            cam_matrix_inv = np.linalg.inv(camera_matrix)

            uv_point = np.array([[clicked_point[0]], [clicked_point[1]], [1]])
            ray_cam = cam_matrix_inv @ uv_point
            ray_world = R_inv @ (ray_cam - tvec)

            scale = -tvec[2] / ray_world[2]
            world_point = R_inv @ (scale * ray_cam - tvec)

            x_mm = world_point[0][0]
            y_mm = world_point[1][0]

            # === OFFSET: Make midpoint of AB = (0, 0) ===
            board_height_mm = (checkerboard_size[1] - 1) * square_size
            x_mm -= board_height_mm / 2

            # === DISPLAY RESULT ===
            cv2.circle(frame, clicked_point, 5, (0, 255, 255), -1)
            cv2.putText(frame, f"({x_mm:.1f}, {y_mm:.1f}) mm",
                        (clicked_point[0] + 10, clicked_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    else:
        cv2.putText(frame, "❌ Checkerboard not found", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # === SHOW FRAME ===
    cv2.imshow("Pixel to mm", frame)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
