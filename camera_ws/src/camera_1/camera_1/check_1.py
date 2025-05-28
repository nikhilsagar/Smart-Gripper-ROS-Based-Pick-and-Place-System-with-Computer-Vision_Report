import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import glob

class CheckerboardViewer(Node):
    def __init__(self):
        super().__init__('checkerboard_viewer')

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open camera")
        else:
            self.get_logger().info("✅ Camera opened successfully")

        # 16x19 squares → 15x18 inner corners
        self.checkerboard_size = (15, 18)
        self.square_size = 25.0  # mm

        self.image_counter = 1
        self.save_dir = "calib_images"
        os.makedirs(self.save_dir, exist_ok=True)

        self.latest_frame_ready = False
        self.latest_frame = None

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("⚠️ Failed to read frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, flags)

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, self.checkerboard_size, corners, ret)
            self.latest_frame_ready = True
            self.latest_frame = frame.copy()
            self.get_logger().info("✅ Checkerboard detected! Press 's' to save image.")
        else:
            self.latest_frame_ready = False
            self.get_logger().info("❌ Checkerboard not found.")

        cv2.imshow("Checkerboard Detection", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

        elif key == ord('s'):
            if self.latest_frame_ready:
                filename = os.path.join(self.save_dir, f"calib_{self.image_counter}.jpg")
                cv2.imwrite(filename, self.latest_frame)
                self.get_logger().info(f"📸 Saved image {self.image_counter} to {filename}")
                self.image_counter += 1
            else:
                self.get_logger().warn("⚠️ Cannot save — checkerboard not detected.")

        elif key == ord('c'):
            self.run_calibration()

    def run_calibration(self):
        self.get_logger().info("🔧 Running camera calibration...")

        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        objpoints = []
        imgpoints = []
        image_size = None

        images = glob.glob(os.path.join(self.save_dir, '*.jpg'))
        if len(images) == 0:
            self.get_logger().error("❌ No calibration images found.")
            return

        for path in images:
            img = cv2.imread(path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)  # Improve contrast

            # ✅ Use smarter corner detector
            ret, corners = cv2.findChessboardCornersSB(gray, self.checkerboard_size, None)

            print(f"🧪 {os.path.basename(path)} — Detected: {ret}")

            if ret:
                image_size = gray.shape[::-1]
                objpoints.append(objp)
                imgpoints.append(corners)
            else:
                fail_path = os.path.join(self.save_dir, f"failed_{os.path.basename(path)}")
                cv2.imwrite(fail_path, img)

            cv2.imshow("Calib View", img)
            cv2.waitKey(50)

        cv2.destroyWindow("Calib View")

        if len(objpoints) == 0:
            self.get_logger().error("❌ No valid checkerboard detections in saved images.")
            return

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None)

        self.get_logger().info("✅ Calibration complete.")
        print("\n📐 Camera Matrix:\n", camera_matrix)
        print("\n🔧 Distortion Coefficients:\n", dist_coeffs)

        np.savez("calibration_data.npz",
                 camera_matrix=camera_matrix,
                 dist_coeffs=dist_coeffs,
                 rvecs=rvecs,
                 tvecs=tvecs)

        self.get_logger().info("💾 Calibration data saved to 'calibration_data.npz'")

def main():
    rclpy.init()
    node = CheckerboardViewer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
