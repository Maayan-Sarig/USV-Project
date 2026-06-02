#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_linear_z', 4)
        self.declare_parameter('axis_angular_z', 3)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('invert_linear_x', True)
        self.declare_parameter('invert_linear_y', False)
        self.declare_parameter('invert_linear_z', False)
        self.declare_parameter('invert_angular_z', False)
        self.declare_parameter('deadzone', 0.1)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Joy to cmd_vel bridge ready.')

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    def joy_callback(self, msg: Joy):
        params = {
            'axis_linear_x': self.get_parameter('axis_linear_x').value,
            'axis_linear_y': self.get_parameter('axis_linear_y').value,
            'axis_linear_z': self.get_parameter('axis_linear_z').value,
            'axis_angular_z': self.get_parameter('axis_angular_z').value,
            'scale_linear': self.get_parameter('scale_linear').value,
            'scale_angular': self.get_parameter('scale_angular').value,
            'invert_linear_x': self.get_parameter('invert_linear_x').value,
            'invert_linear_y': self.get_parameter('invert_linear_y').value,
            'invert_linear_z': self.get_parameter('invert_linear_z').value,
            'invert_angular_z': self.get_parameter('invert_angular_z').value,
            'deadzone': self.get_parameter('deadzone').value,
        }

        twist = Twist()

        def axis_value(index):
            if index < 0 or index >= len(msg.axes):
                return 0.0
            return msg.axes[index]

        if params['invert_linear_x']:
            twist.linear.x = -axis_value(params['axis_linear_x']) * params['scale_linear']
        else:
            twist.linear.x = axis_value(params['axis_linear_x']) * params['scale_linear']

        if params['invert_linear_y']:
            twist.linear.y = -axis_value(params['axis_linear_y']) * params['scale_linear']
        else:
            twist.linear.y = axis_value(params['axis_linear_y']) * params['scale_linear']

        if params['invert_linear_z']:
            twist.linear.z = -axis_value(params['axis_linear_z']) * params['scale_linear']
        else:
            twist.linear.z = axis_value(params['axis_linear_z']) * params['scale_linear']

        if params['invert_angular_z']:
            twist.angular.z = -axis_value(params['axis_angular_z']) * params['scale_angular']
        else:
            twist.angular.z = axis_value(params['axis_angular_z']) * params['scale_angular']

        twist.linear.x = self.apply_deadzone(twist.linear.x, params['deadzone'])
        twist.linear.y = self.apply_deadzone(twist.linear.y, params['deadzone'])
        twist.linear.z = self.apply_deadzone(twist.linear.z, params['deadzone'])
        twist.angular.z = self.apply_deadzone(twist.angular.z, params['deadzone'])

        self.pub.publish(twist)
        self.get_logger().debug(f'Published cmd_vel: {twist}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
