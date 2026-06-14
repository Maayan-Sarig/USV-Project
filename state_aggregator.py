#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


class USVStateNode(Node):
    def __init__(self):
        super().__init__('usv_state_aggregator')
        self.state_lock = threading.Lock()
        self.state = {
            'gps': None,
            'imu_location': None,
            'tension': None,
            'encoder_angle': None,
            't200_speed': None,
            'stepper': None,
            'battery': None,
            'rov_position': None,
            'usv_mode': 'STATION_KEEPING',
            'rtl_active': False,
            'last_update': None,
        }

        self.create_subscription(Float32MultiArray, 'gps', self.on_gps, 10)
        self.create_subscription(Float32MultiArray, 'location', self.on_location, 10)
        self.create_subscription(Float32, 'tension', self.on_tension, 10)
        self.create_subscription(Float32, 'encoder_angle', self.on_encoder, 10)
        self.create_subscription(Float32MultiArray, 't200_speed', self.on_thruster, 10)
        self.create_subscription(Float32, 'stepper', self.on_stepper, 10)
        self.create_subscription(Float32MultiArray, 'battery', self.on_battery, 10)
        self.create_subscription(Float32MultiArray, 'rov_position', self.on_rov_position, 10)
        self.create_subscription(String,            'usv_mode',     self.on_usv_mode,     10)
        self.create_subscription(String,            'rtl_trigger',  self.on_rtl_trigger,  10)

        self.get_logger().info('USV state aggregator started.')

    def set_state(self, key, value):
        with self.state_lock:
            self.state[key] = value
            self.state['last_update'] = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def on_gps(self, msg: Float32MultiArray):
        self.set_state('gps', list(msg.data))

    def on_location(self, msg: Float32MultiArray):
        self.set_state('imu_location', list(msg.data))

    def on_tension(self, msg: Float32):
        self.set_state('tension', float(msg.data))

    def on_encoder(self, msg: Float32):
        self.set_state('encoder_angle', float(msg.data))

    def on_thruster(self, msg: Float32MultiArray):
        self.set_state('t200_speed', list(msg.data))

    def on_stepper(self, msg: Float32):
        self.set_state('stepper', float(msg.data))

    def on_battery(self, msg: Float32MultiArray):
        self.set_state('battery', list(msg.data))

    def on_rov_position(self, msg: Float32MultiArray):
        self.set_state('rov_position', list(msg.data))

    def on_usv_mode(self, msg: String):
        self.set_state('usv_mode', msg.data)
        self.set_state('rtl_active', msg.data == 'RTL')

    def on_rtl_trigger(self, msg: String):
        self.set_state('rtl_active', True)

    def get_state(self):
        with self.state_lock:
            return dict(self.state)


def main(args=None):
    rclpy.init(args=args)
    node = USVStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
