import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ROS 2 Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/aruco_detected', 10)
        self.coord_publisher = self.create_publisher(String, 'aruco/coordinates', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Open Camera
        self.cap = cv2.VideoCapture(0)

        # ArUco Dictionary & Detector Parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Timer to process frames
        self.timer = self.create_timer(0.1, self.process_frame)  # 10Hz

    def process_frame(self):
        """ Detect ArUco markers and publish coordinates """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                corner_pts = corners[i][0]  # Extract the four corner points

                # Calculate the center of the marker
                cX = int(np.mean(corner_pts[:, 0]))
                cY = int(np.mean(corner_pts[:, 1]))

                # Draw the bounding box and marker ID
                cv2.polylines(frame, [np.int32(corner_pts)], True, (0, 255, 0), 2)
                cv2.putText(frame, f"ID: {ids[i][0]}", (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Publish coordinates as a ROS 2 message
                coord_msg = String()
                coord_msg.data = f"Marker {ids[i][0]}: ({cX}, {cY})"
                self.coord_publisher.publish(coord_msg)
                self.get_logger().info(coord_msg.data)

        # Publish the processed image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(img_msg)

        # Display the result
        cv2.imshow("Aruco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
