import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class MultiColorObjectDetector(Node):
    def __init__(self):
        super().__init__('multi_color_object_detector')

        # Load square data
        package_share = get_package_share_directory('camera_1')
        self.package_share = package_share
        json_path = os.path.join(package_share, 'resource', 'squares_config.json')
        with open(json_path, 'r') as f:
            self.square_data = json.load(f)['squares']
        self.get_logger().info("📁 Loaded squares_config.json successfully")

        # HSV defaults
        self.default_hsv_values = {
            "Red": [0, 124, 63, 6, 255, 255],
            "Yellow": [17, 34, 139, 67, 255, 255],
            "Blue": [83, 70, 98, 120, 228, 255],
            "Green": [55, 118, 59, 78, 255, 255]
        }

        self.bgr_colors = {
            "Red": (0, 0, 255),
            "Yellow": (0, 255, 255),
            "Blue": (255, 0, 0),
            "Green": (0, 255, 0)
        }

        self.color_priority = {
            "Red": 0,
            "Green": 1,
            "Blue": 2,
            "Yellow": 3
        }

        self.create_trackbar_windows()
        self.top_object_publisher = self.create_publisher(String, 'top_priority_object', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open camera")
            rclpy.shutdown()
            return

        self.get_logger().info("✅ Camera opened successfully")
        self.timer = self.create_timer(0.1, self.detect_objects)

    def create_trackbar_windows(self):
        for color, (h_l, s_l, v_l, h_u, s_u, v_u) in self.default_hsv_values.items():
            cv2.namedWindow(f"HSV - {color}")
            cv2.createTrackbar("H Lower", f"HSV - {color}", h_l, 179, lambda x: None)
            cv2.createTrackbar("S Lower", f"HSV - {color}", s_l, 255, lambda x: None)
            cv2.createTrackbar("V Lower", f"HSV - {color}", v_l, 255, lambda x: None)
            cv2.createTrackbar("H Upper", f"HSV - {color}", h_u, 179, lambda x: None)
            cv2.createTrackbar("S Upper", f"HSV - {color}", s_u, 255, lambda x: None)
            cv2.createTrackbar("V Upper", f"HSV - {color}", v_u, 255, lambda x: None)

    def get_hsv_values(self, color):
        h_l = cv2.getTrackbarPos("H Lower", f"HSV - {color}")
        s_l = cv2.getTrackbarPos("S Lower", f"HSV - {color}")
        v_l = cv2.getTrackbarPos("V Lower", f"HSV - {color}")
        h_u = cv2.getTrackbarPos("H Upper", f"HSV - {color}")
        s_u = cv2.getTrackbarPos("S Upper", f"HSV - {color}")
        v_u = cv2.getTrackbarPos("V Upper", f"HSV - {color}")
        return np.array([h_l, s_l, v_l]), np.array([h_u, s_u, v_u])

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_objects = []

        for color in ['Red', 'Green', 'Blue', 'Yellow']:
            lower, upper = self.get_hsv_values(color)

            if color == 'Red':
                lower1 = np.array([0, lower[1], lower[2]])
                upper1 = np.array([6, upper[1], upper[2]])
                lower2 = np.array([170, lower[1], lower[2]])
                upper2 = np.array([179, upper[1], upper[2]])
                mask1 = cv2.inRange(hsv, lower1, upper1)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, lower, upper)

            cv2.imshow(f"Mask - {color}", mask)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 300:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 6, self.bgr_colors[color], -1)

                min_dist = float('inf')
                closest_id = None
                for square_id, square_info in self.square_data.items():
                    px, py = square_info["pixel_center"]
                    dist = np.linalg.norm(np.array([cx, cy]) - np.array([px, py]))
                    if dist < min_dist:
                        min_dist = dist
                        closest_id = square_id

                if closest_id is not None:
                    label = f"{color} ID: {closest_id}"
                    cv2.putText(frame, label, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.bgr_colors[color], 2)
                    detected_objects.append((int(closest_id), color))

        detected_objects.sort(key=lambda x: (self.color_priority[x[1]], x[0]))

        if detected_objects:
            print("\n📦 Sorted Detected Objects:")
            for obj_id, color in detected_objects:
                print(f"ID: {obj_id}, Color: {color}")
        else:
            print("\n📦 No objects detected")

        if detected_objects:
            top_id, top_color = detected_objects[0]
            msg = String()
            msg.data = f"ID: {top_id}, Color: {top_color}"
            self.top_object_publisher.publish(msg)

        cv2.imshow("Multi-Color Object Detection", frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MultiColorObjectDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()
