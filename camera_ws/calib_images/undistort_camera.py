import cv2
import numpy as np

# Load calibration data
with np.load("calibration_data.npz") as data:
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']

# Start camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("❌ Failed to open camera.")
    exit()

print("✅ Camera opened. Showing undistorted feed...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to grab frame")
        break

    # Undistort frame
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # Show both frames
    combined = np.hstack((frame, undistorted))
    cv2.imshow("Original (left) vs Undistorted (right)", combined)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

