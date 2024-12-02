#TO DO: Angie only provides the model
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

import os



class ArtificialVisionNode(Node):
    def __init__(self):
        super().__init__('artificial_vision_node')
        
        # Publicadores
        self.detection_publisher = self.create_publisher(Detection2DArray, '/yolo/detecttion', 10)
        #Subcription to camera
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.process_frame,
            10
        )
        self.bridge = CvBridge()
        # Modelo YOLO
        self.model = YOLO("/home/josflame11/robot_ws/src/robot_behavior/robot_behavior/weights2/best_ncnn_model", 
                          task='detect')
        
        #Map for classes
        self.YOLO_CLASSES = {
                        0: "human",
                        1: "cross",
                        2: "stop"
                        }
        
        self.get_logger().info("Artificial Vision Node started.")
    
    def process_frame(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            results = self.model(frame)
            detections = results[0].boxes

            detection_array_msg = Detection2DArray()
            
            for detection in detections:
                detection_msg = Detection2D()
                
                hypothesis = ObjectHypothesisWithPose()
                class_id = int(detection.cls.item())
                label = self.YOLO_CLASSES.get(class_id, "Unknown")

                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = float(detection.conf.item())

                detection_msg = Detection2D()
                detection_msg.results.append(hypothesis)
                detection_array_msg.detections.append(detection_msg)

            self.detection_publisher.publish(detection_array_msg)
            self.get_logger().info(f"Published {len(detections)} detections.")

            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
                

def main(args=None):
    rclpy.init(args=args)
    node = ArtificialVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("STOP")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()