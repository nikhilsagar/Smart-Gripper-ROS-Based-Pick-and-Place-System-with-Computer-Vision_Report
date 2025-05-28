import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import os

class CheckerboardLabelerWithCoords(Node):
    def __init__(self):
        super().__init__('checkerboard_labeler_coords')

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.checkerboard_size = (15, 18)  # (cols, rows) = (15, 18) → 14x17 squares
        self.square_size_mm = 22.68
        self.save_path = "squares_config.json"

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("⚠️ Could not read frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(gray, self.checkerboard_size, None)

        square_data = {}
        if ret:
            self.get_logger().info("✅ Checkerboard detected")

            corners = corners.reshape(self.checkerboard_size[1], self.checkerboard_size[0], 2)
            square_id = 0
            rows = self.checkerboard_size[1] - 1
            cols = self.checkerboard_size[0] - 1

            origin_row = rows // 2  # center of left column
            for row in range(rows):
                for col in range(cols):
                    tl = corners[row][col]
                    tr = corners[row][col + 1]
                    bl = corners[row + 1][col]
                    br = corners[row + 1][col + 1]

                    center_x = int((tl[0] + tr[0] + bl[0] + br[0]) / 4)
                    center_y = int((tl[1] + tr[1] + bl[1] + br[1]) / 4)

                    x_mm = col * self.square_size_mm
                    y_mm = (origin_row - row) * self.square_size_mm  # y increases going up

                    square_data[square_id] = {
                        "pixel_center": [center_x, center_y],
                        "real_world": {
                            "x": round(x_mm, 2),
                            "y": round(y_mm, 2)
                        }
                    }

                    cv2.putText(frame, str(square_id), (center_x - 10, center_y + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    square_id += 1

            # Save the data
            with open(self.save_path, 'w') as f:
                json.dump({"squares": square_data}, f, indent=4)
            self.get_logger().info(f"💾 Saved square data to {self.save_path}")

        else:
            self.get_logger().info("❌ Checkerboard not found")

        cv2.imshow("Checkerboard with Coordinates", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = CheckerboardLabelerWithCoords()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
