#TO DO: ANGIE
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge

class LineFollower(Node):
    def __init__(self):
        super().__init__('path_tracking')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.get_logger().info("LineFollower node has been started.")

    def image_callback(self, msg):
        try:
            # Convert CompressedImage to OpenCV image
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            # Process the image to detect the line and calculate offset
            frame_with_lines, offset = self.detect_line_and_offset(frame)

            # Compute Twist message based on the offset
            twist = Twist()
            twist.linear.x = 0.5  # Set a constant forward speed
            twist.angular.z = -0.1 * offset  # Proportional control for angular velocity
            self.publisher.publish(twist)

            # Optional: display the processed image for debugging
            cv2.imshow("Line Detection", frame_with_lines)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def detect_line_and_offset(self, frame):
        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Define a region of interest
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # Triangle region for detecting the road line
        polygon = np.array([[
            (width  * 1.0, height * 1.0),
            (width * 0.0, height * 1.0),
            (width * 0.0, height * 1.0),
            (width * 0.3, height * 0.55),
            (width * 0.7, height * 0.55),
            (width * 1.0, height * 1.0),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detect lines using Hough Transformation
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 100, minLineLength=40, maxLineGap=50)

        # Center of the image
        center_x = width // 2

        # Calculate the average position of detected lines
        line_positions = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
                line_positions.extend([x1, x2])

        # Calculate the offset
        if line_positions:
            average_x = sum(line_positions) / len(line_positions)
            offset = average_x - center_x
            scaled_offset = np.interp(offset, [-center_x, center_x], [-6, 8])
        else:
            scaled_offset = 0

        # Display the offset on the image
        cv2.putText(frame, f"Offset: {scaled_offset:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        return frame, scaled_offset


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
