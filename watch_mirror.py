#!/usr/bin/env python3
"""One-terminal watcher: prints rov_flight_mode, rov_manual_control,
mirror_angle, mirror_boost, and t200_speed as they update, side by side."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


class Watch(Node):
    def __init__(self):
        super().__init__('watch_mirror')
        self.create_subscription(String, 'rov_flight_mode', lambda m: self._p('rov_flight_mode', m.data), 10)
        self.create_subscription(Float32MultiArray, 'rov_manual_control', lambda m: self._p('rov_manual_control', list(m.data)), 10)
        self.create_subscription(Float32, 'mirror_angle', lambda m: self._p('mirror_angle', round(m.data, 1)), 10)
        self.create_subscription(Float32, 'mirror_boost', lambda m: self._p('mirror_boost', round(m.data, 1)), 10)
        self.create_subscription(Float32MultiArray, 't200_speed', lambda m: self._p('t200_speed', list(m.data)), 10)

    def _p(self, name, value):
        print(f'{name:20s} {value}')


def main():
    rclpy.init()
    node = Watch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
