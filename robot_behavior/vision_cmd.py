import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class VisionCmd(Node):
    def __init__(self):
        super().__init__('vision_cmd_node')
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detecttion',
            self.detection_callback,
            1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/vision', 10)

        self.active_state_publisher = self.create_publisher(Bool, '/vision_cmd/active', 10)

        self.low_speed = 0.05

        self.get_logger().info("Vision Command node started")

    def detection_callback(self, msg):
        stop_detected = False
        human_detected = False
        cross_detected = False

        for detection in msg.detections:
            for result in detection.results:
                label = result.hypothesis.class_id
                score = result.hypothesis.score

                if label == "stop" and score > 0.8:
                    stop_detected = True
                    break 

                if label == "cross" and score > 0.5:
                    cross_detected = True

                if label == "human" and score > 0.5:
                    human_detected = True

            if stop_detected:
                break

        # Publish the active state flag
        active_state_msg = Bool()
        active_state_msg.data = stop_detected or cross_detected or (human_detected and cross_detected)
        self.active_state_publisher.publish(active_state_msg)

        if stop_detected:
            self.get_logger().info("Stopping")
            self.publish_vel(0.0)
        elif cross_detected and human_detected:
            self.get_logger().info("Stopping for human")
            self.publish_vel(0.0)
        elif cross_detected:
            self.get_logger().info("Lowering speed")
            self.publish_vel(self.low_speed)
        else:
            self.get_logger().info("Continue following the path")

    def publish_vel(self, speed):
        vel = Twist()
        vel.linear.x = speed
        self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = VisionCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    