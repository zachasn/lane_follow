import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32


class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        # Gain for proportional control
        self.kp = 0.5   # <-- higher gain because error is normalized

        # Publisher to cmd_vel (TwistStamped)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Subscriber to lane error
        self.error_sub = self.create_subscription(
            Float32,
            '/lane_error',
            self.error_callback,
            10
        )

        self.get_logger().info('Lane controller node started.')

    def error_callback(self, msg):
        error = msg.data

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        twist.twist.linear.x = 0.2
        twist.twist.angular.z = -self.kp * error

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
