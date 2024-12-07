import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        self.odom_sub = self.create_subscription(Odometry, '/minimal_controller/odom', self.update_position, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel/go2goal', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.k1 = 0.094315  # linear gain
        self.k2 = 2.9842 # angular gain

        self.goal_x = float(input("Set your x goal position: "))
        self.goal_y = float(input("Set your y goal position: "))

        self.distance_tolerance = 0.1
        self.goal_reach = False

    def update_position(self, msg):
        self.x = round(msg.pose.pose.position.x, 4)
        self.y = round(msg.pose.pose.position.y, 4)
        
        # Quaternion to yaw (theta)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        if not self.goal_reach:
            self.move2goal()
        
    
    def get_distance_to_goal(self, goal_x, goal_y):
        return math.sqrt(pow(self.x - goal_x, 2) + pow(self.y - goal_y, 2))
    
    def get_angle_to_goal(self, goal_x, goal_y):
        return math.atan2(goal_y - self.y , goal_x - self.x)
    
    def move2goal(self):
        # Get Goal position

        vel_msg = Twist()

        distance_to_goal = self.get_distance_to_goal(self.goal_x, self.goal_y)
        angular_error = self.get_angle_to_goal(self.goal_x, self.goal_y)  - self.theta

        # Normalize angular error to [-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))


        #Controller for navigation
        if distance_to_goal > self.distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 2.5 #self.k1 * distance_to_goal #Change to fixed value for only tracking the angular position

            # Angular velocity in the z-axis.
            vel_msg.angular.z = self.k2 * angular_error

            # Publishing our vel_msg
            self.cmd_vel_pub.publish(vel_msg)

            # Debugging log
            self.get_logger().info(f'Current Pose: x={self.x}, y={self.y}, distance={distance_to_goal:.4f}')

        else:
            # Stopping our robot after the loop
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.cmd_vel_pub.publish(vel_msg)
            # Mark goal as reached
            self.goal_reached = True
            self.get_logger().info("Goal reached!")

            if rclpy.ok():
                # Shut down the node
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
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