import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ContourDetectionNode(Node):
    def __init__(self):
        super().__init__('contour_detection_node')

        # Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.coord_publisher = self.create_publisher(String, 'detected_objects', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Create separate trackbar windows for each color
        self.create_trackbar_windows()

        # Timer for periodic frame capture
        self.timer = self.create_timer(0.1, self.process_frame)  # 10Hz

    def create_trackbar_windows(self):
        """ Create separate trackbars for each color's HSV tuning. """
        colors = ["Red", "Blue", "Green", "Yellow"]
        for color in colors:
            cv2.namedWindow(f"HSV Adjustments - {color}")

            cv2.createTrackbar(f"H Lower", f"HSV Adjustments - {color}", 0, 179, lambda x: None)
            cv2.createTrackbar(f"S Lower", f"HSV Adjustments - {color}", 100, 255, lambda x: None)
            cv2.createTrackbar(f"V Lower", f"HSV Adjustments - {color}", 100, 255, lambda x: None)
            cv2.createTrackbar(f"H Upper", f"HSV Adjustments - {color}", 179, 179, lambda x: None)
            cv2.createTrackbar(f"S Upper", f"HSV Adjustments - {color}", 255, 255, lambda x: None)
            cv2.createTrackbar(f"V Upper", f"HSV Adjustments - {color}", 255, 255, lambda x: None)

    def get_hsv_values(self, color_name):
        """ Read trackbar values for a given color. """
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

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get HSV values from trackbars
        color_ranges = {
            "red": self.get_hsv_values("Red"),
            "blue": self.get_hsv_values("Blue"),
            "green": self.get_hsv_values("Green"),
            "yellow": self.get_hsv_values("Yellow"),
        }

        detected_objects = []

        # Iterate through each color
        for color, hsv_range in color_ranges.items():
            lower, upper = hsv_range
            mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Assign contour and text colors
            if color == "red":
                contour_color = (0, 0, 255)  # Red
                text_color = (0, 0, 255)
            elif color == "blue":
                contour_color = (255, 0, 0)  # Blue
                text_color = (255, 0, 0)
            elif color == "green":
                contour_color = (0, 255, 0)  # Green
                text_color = (0, 255, 0)
            elif color == "yellow":
                contour_color = (0, 255, 255)  # Yellow
                text_color = (0, 255, 255)

            # Process detected contours
            for cnt in contours:
                area = cv2.contourArea(cnt)

                # Filter out small objects (area < 500 pixels)
                if area < 500:
                    continue

                # Get bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2  # Object center

                # Draw contour and label
                cv2.drawContours(frame, [cnt], -1, contour_color, 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), contour_color, 2)
                cv2.putText(frame, color.capitalize(), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

                # Store detected object position
                detected_objects.append(f"{color.capitalize()}: ({cx}, {cy})")

        # Publish the processed image to ROS 2 topic for RViz
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(img_msg)

        # Publish detected object coordinates
        if detected_objects:
            coord_msg = String()
            coord_msg.data = ", ".join(detected_objects)
            self.coord_publisher.publish(coord_msg)
            self.get_logger().info(f"Detected objects: {coord_msg.data}")

        # Show original frame and mask
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)  # Needed to update OpenCV windows

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ContourDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
