import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import os
from ament_index_python.packages import get_package_share_directory

class GreenObjectDetector(Node):
    def __init__(self):
        super().__init__('green_object_detector')

        # ✅ Load squares_config.json from package resource folder
        package_share = get_package_share_directory('camera_1')
        json_path = os.path.join(package_share, 'resource', 'squares_config.json')

        with open(json_path, 'r') as f:
            self.square_data = json.load(f)['squares']

        self.get_logger().info("📁 Loaded squares_config.json successfully")

        # HSV range for green detection
        self.lower_green = np.array([40, 70, 70])
        self.upper_green = np.array([85, 255, 255])

        # Open camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open camera")
            rclpy.shutdown()
            return

        self.get_logger().info("✅ Camera opened successfully")
        self.timer = self.create_timer(0.1, self.detect_green_object)

    def detect_green_object(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Draw object center
            cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

            # Find closest square
            min_dist = float('inf')
            closest_id = None

            for square_id, square_info in self.square_data.items():
                px, py = square_info["pixel_center"]
                dist = np.linalg.norm(np.array([cx, cy]) - np.array([px, py]))

                if dist < min_dist:
                    min_dist = dist
                    closest_id = square_id

            if closest_id is not None:
                cv2.putText(frame, f"🟢 ID: {closest_id}", (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self.get_logger().info(f"✅ Green object at Square ID: {closest_id}")

        # Show frame
        cv2.imshow("Green Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GreenObjectDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()
