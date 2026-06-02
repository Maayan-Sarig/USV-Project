import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.thruster_pub = self.create_publisher(Float32, 'thruster_cmd', 10)
        self.boost_pub = self.create_publisher(Float32, 'follow', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('cmd_vel bridge ready. Publish geometry_msgs/Twist on cmd_vel.')

    def cmd_vel_callback(self, msg: Twist):
        # linear.x = forward/backward, linear.y ignored, linear.z could be used for depth control.
        # The thruster node expects a Float32 command in roughly the same range as its current encoder_angle input.
        thrust = max(-100.0, min(100.0, float(msg.linear.x) * 20.0))
        boost = max(-50.0, min(50.0, float(msg.angular.z) * 20.0))

        self.thruster_pub.publish(Float32(data=thrust))
        self.boost_pub.publish(Float32(data=boost))

        self.get_logger().debug(f'cmd_vel -> thruster_cmd={thrust:.2f} follow={boost:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
