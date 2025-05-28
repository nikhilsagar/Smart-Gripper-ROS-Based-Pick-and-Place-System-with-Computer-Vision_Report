import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class ArucoContourDetector(Node):
    def __init__(self):
        super().__init__('aruco_contour_detector')

        # ROS 2 Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/filtered_objects', 10)
        self.coord_publisher = self.create_publisher(String, 'objects/real_world_coords', 10)
        self.top_object_publisher = self.create_publisher(String, 'top_priority_object', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Open Camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # ArUco Dictionary & Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

        # Updated HSV Defaults
        self.default_hsv_values = {
            "Yellow": (16, 96, 146, 28, 255, 255),
            "Red": (0, 197, 158, 179, 255, 254),
            "Green": (50, 86, 99, 93, 254, 255),
            "Blue": (98, 62, 78, 134, 255, 255)
        }

        self.create_trackbar_windows()
        self.timer = self.create_timer(0.1, self.process_frame)

    def create_trackbar_windows(self):
        for color, (h_l, s_l, v_l, h_u, s_u, v_u) in self.default_hsv_values.items():
            cv2.namedWindow(f"HSV Adjustments - {color}")
            cv2.createTrackbar(f"H Lower", f"HSV Adjustments - {color}", h_l, 179, lambda x: None)
            cv2.createTrackbar(f"S Lower", f"HSV Adjustments - {color}", s_l, 255, lambda x: None)
            cv2.createTrackbar(f"V Lower", f"HSV Adjustments - {color}", v_l, 255, lambda x: None)
            cv2.createTrackbar(f"H Upper", f"HSV Adjustments - {color}", h_u, 179, lambda x: None)
            cv2.createTrackbar(f"S Upper", f"HSV Adjustments - {color}", s_u, 255, lambda x: None)
            cv2.createTrackbar(f"V Upper", f"HSV Adjustments - {color}", v_u, 255, lambda x: None)

    def get_hsv_values(self, color_name):
        h_lower = cv2.getTrackbarPos(f"H Lower", f"HSV Adjustments - {color_name}")
        s_lower = cv2.getTrackbarPos(f"S Lower", f"HSV Adjustments - {color_name}")
        v_lower = cv2.getTrackbarPos(f"V Lower", f"HSV Adjustments - {color_name}")
        h_upper = cv2.getTrackbarPos(f"H Upper", f"HSV Adjustments - {color_name}")
        s_upper = cv2.getTrackbarPos(f"S Upper", f"HSV Adjustments - {color_name}")
        v_upper = cv2.getTrackbarPos(f"V Upper", f"HSV Adjustments - {color_name}")
        return [h_lower, s_lower, v_lower], [h_upper, s_upper, v_upper]

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)

        marker_positions = {}
        midpoint = (0, 0)

        if ids is not None and len(ids) >= 4:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                corner_pts = corners[i][0]
                cX, cY = int(np.mean(corner_pts[:, 0])), int(np.mean(corner_pts[:, 1]))
                marker_positions[marker_id] = (cX, cY)

                cv2.polylines(frame, [np.int32(corner_pts)], True, (255, 105, 180), 2)
                cv2.putText(frame, f"ID: {marker_id}", (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 105, 180), 2)

            if all(k in marker_positions for k in [0, 1, 2, 3]):
                cv2.line(frame, marker_positions[0], marker_positions[1], (255, 255, 0), 2)
                cv2.line(frame, marker_positions[1], marker_positions[3], (255, 255, 0), 2)
                cv2.line(frame, marker_positions[3], marker_positions[2], (255, 255, 0), 2)
                cv2.line(frame, marker_positions[2], marker_positions[0], (255, 255, 0), 2)

                midpoint = ((marker_positions[0][0] + marker_positions[1][0]) // 2,
                            (marker_positions[0][1] + marker_positions[1][1]) // 2)
                cv2.circle(frame, midpoint, 5, (0, 255, 255), -1)

        self.marker_positions = marker_positions
        self.detect_colored_objects(frame, midpoint)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(img_msg)

        cv2.imshow("Filtered Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def detect_colored_objects(self, frame, midpoint):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_ranges = {
            "Red": (self.get_hsv_values("Red"), (0, 0, 255)),
            "Blue": (self.get_hsv_values("Blue"), (255, 0, 0)),
            "Green": (self.get_hsv_values("Green"), (0, 255, 0)),
            "Yellow": (self.get_hsv_values("Yellow"), (0, 255, 255))
        }

        pixels_per_mm_x = None
        pixels_per_mm_y = None

        if hasattr(self, "marker_positions") and all(k in self.marker_positions for k in [0, 1, 3]):
            x0, y0 = self.marker_positions[0]
            x1, y1 = self.marker_positions[1]
            x3, y3 = self.marker_positions[3]

            pixel_dist_x = np.linalg.norm(np.array([x1, y1]) - np.array([x0, y0]))
            pixel_dist_y = np.linalg.norm(np.array([x3, y3]) - np.array([x1, y1]))

            pixels_per_mm_x = pixel_dist_x / 35.0
            pixels_per_mm_y = pixel_dist_y / 55.0

        if not hasattr(self, "detected_objects"):
            self.detected_objects = []
        self.detected_objects.clear()

        priority_order = {"Red": 0, "Yellow": 1, "Blue": 2, "Green": 3}

        for color, (hsv_range, color_bgr) in color_ranges.items():
            lower, upper = hsv_range
            mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            cv2.imshow(f"Mask - {color.capitalize()}", mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1000:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2

                cv2.drawContours(frame, [cnt], -1, color_bgr, 2)
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                cv2.putText(frame, color.capitalize(), (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

                cv2.line(frame, midpoint, (cx, cy), color_bgr, 2)

                if pixels_per_mm_x and pixels_per_mm_y:
                    dy_pix = cx - midpoint[0]
                    dx_pix = cy - midpoint[1]
                    x_mm = dx_pix / pixels_per_mm_y
                    y_mm = dy_pix / pixels_per_mm_x

                    label = f"({x_mm:.1f}, {y_mm:.1f}) mm"
                    cv2.putText(frame, label, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

                    self.detected_objects.append({
                        "color": color,
                        "x": round(x_mm, 2),
                        "y": round(y_mm, 2),
                        "priority": priority_order[color]
                    })

        self.detected_objects.sort(key=lambda obj: obj["priority"])

        if self.detected_objects:
            print("\n📦 Sorted Detected Objects (by priority):")
            for obj in self.detected_objects:
                print(obj)

            top_object = self.detected_objects[0]
        else:
            top_object = {"color": "no_data", "x": 0.0, "y": 0.0}

        self.top_object_publisher.publish(String(data=json.dumps(top_object)))


def main():
    rclpy.init()
    node = ArucoContourDetector()
    rclpy.spin(node)
    rclpy.shutdown()
