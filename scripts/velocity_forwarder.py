#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityForwarder(Node):
    def __init__(self):
        super().__init__('velocity_forwarder')
        # Subscriber for /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for /diff_cont/cmd_vel_unstamped
        self.publisher = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10)
        self.get_logger().info("VelocityForwarder node started, waiting for /cmd_vel messages...")

    def cmd_vel_callback(self, msg: Twist):
        # Log receipt of the message
        self.get_logger().info("Received /cmd_vel message, forwarding to /diff_cont/cmd_vel_unstamped")
        # Publish the received message on /diff_cont/cmd_vel_unstamped
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
