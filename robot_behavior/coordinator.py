import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        # Subscriptions to /cmd_vel topics from the nodes
        self.create_subscription(Twist, '/cmd_vel/sensors', self.sensor_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel/follower', self.path_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel/vision', self.vision_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel/go2goal', self.gotogoal_cmd_callback, 10)

        # Subscriptions to state flags
        self.create_subscription(Bool, '/sensor_avoidance/active', self.sensor_active_callback, 10)
        self.create_subscription(Bool, '/vision_cmd/active', self.vision_active_callback, 10)

        # Publisher for final /cmd_vel topic
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Priority flags
        self.sensor_active = False
        self.vision_active = False

        # Latest velocity messages
        self.sensor_cmd = None
        self.path_cmd = None
        self.vision_cmd = None
        self.gotogoal_cmd = None

        self.get_logger().info("Coordinator Node started")

    def sensor_cmd_callback(self, msg):
        self.sensor_cmd = msg

    def path_cmd_callback(self, msg):
        self.path_cmd = msg

    def vision_cmd_callback(self, msg):
        self.vision_cmd = msg

    def gotogoal_cmd_callback(self, msg):
        self.gotogoal_cmd = msg

    def sensor_active_callback(self, msg):
        self.sensor_active = msg.data

    def vision_active_callback(self, msg):
        self.vision_active = msg.data

    def publish_cmd(self):
        # Determine which command to publish based on priorities
        if self.vision_active and self.vision_cmd is not None:
            self.cmd_pub.publish(self.vision_cmd)
            self.get_logger().info("Publishing Vision Command")
        elif self.sensor_active and self.sensor_cmd is not None:
            self.cmd_pub.publish(self.sensor_cmd)
            self.get_logger().info("Publishing Sensor Avoidance Command")
        elif self.gotogoal_cmd is not None:
            self.cmd_pub.publish(self.gotogoal_cmd)
            self.get_logger().info("Publishing GoToGoal Command")
        elif self.path_cmd is not None:
            self.cmd_pub.publish(self.path_cmd)
            self.get_logger().info("Publishing Path Tracking Command")
        else:
            # Default stop command if no input is available
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().info("Publishing Stop Command")

    def run(self):
        # Timer to continuously evaluate and publish commands
        self.create_timer(0.1, self.publish_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Coordinator Node stopped cleanly")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
