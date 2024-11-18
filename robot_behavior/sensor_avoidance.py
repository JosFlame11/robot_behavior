import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Laser
from std_msgs.msg import Int8

class ObjectAvoidance(Node):
    def __init__(self):
        super().__init__('sensor_object_avoidance')

        self.left_sensor_subscriber = self.create_subscription(
            Int8,
            '/DS3/state',
            self.left_sensor_callback,
            10
        )
        self.right_sensor_subscriber = self.create_subscription(
            Int8,
            '/DS1/state',
            self.right_sensor_callback,
            10
        )
        self.front_sensor_susbcriber = self.create_subscription(
            Int8,
            '/DS2/state',
            self.front_sensor_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            #'cmd_vel_sensors',
            10
        )

        self.is_object_left = False
        self.is_object_right = False
        self.is_object_front = False

        self.MAX_LINEAR_VEL = 0.25
        self.MAX_ANGULAR_VEL = 1.0

        self.publish_cmd_vel

    def left_sensor_callback(self, msg):
        # Check if there is an obstacle in the left sensor
        if msg.data == 0:
            self.is_object_left = True
        else:
            self.is_object_left = False
        self.publish_cmd_vel()

    def right_sensor_callback(self, msg):
        # Check if there is an obstacle in the right sensor
        if msg.data == 0:
            self.is_object_right = True
        else:
            self.is_object_right = False
        self.publish_cmd_vel()

    def front_sensor_callback(self, msg):
        # Check if there is an obstacle in the front sensor
        if msg.data == 0:
            self.is_object_front = True
        else:
            self.is_object_front = False
        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        # Publish a Twist message to the cmd_vel topic
        twist = Twist()
        if self.is_object_front:
            twist.linear.x = 0.0
            # self.get_logger().info("Front")
            twist.angular.z = self.MAX_ANGULAR_VEL

        elif self.is_object_left:
            # self.get_logger().info("Left")
            twist.linear.x = self.MAX_LINEAR_VEL * 0.4
            twist.angular.z = -self.MAX_ANGULAR_VEL

        elif self.is_object_right:
            # self.get_logger().info("Right")
            twist.linear.x = self.MAX_LINEAR_VEL * 0.4
            twist.angular.z = self.MAX_ANGULAR_VEL

        elif self.is_object_right and self.is_object_front:
            twist.linear.x = self.MAX_LINEAR_VEL * 0.4
            twist.angular.z = self.MAX_ANGULAR_VEL

        else:
            twist.linear.x = self.MAX_LINEAR_VEL
            twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    object_avoidance = ObjectAvoidance()

    try:
        rclpy.spin(object_avoidance)
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')
    finally:
        if rclpy.ok():
            object_avoidance.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()