#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Import the steering dead zone so cruise activates in exactly the same angle
# band where proportional steering does nothing — single source of truth.
from t200 import DEAD_ZONE_DEG

# Tune these for the specific ROV survey speed and cable characteristics
LOW_TENSION   = 20    # kg — lower bound of nominal tension band
HIGH_TENSION  = 35    # kg — upper bound of nominal tension band
CRUISE_BOOST  = 30    # µs added to both T200 motors when cruising forward


class CruiseControlNode(Node):
    """
    Publishes a constant forward boost on 'cruise_boost' when the USV should
    track the ROV at a steady speed (e.g. a straight-line survey pass).

    Activation conditions (both must be true simultaneously):
      - Cable angle is within ±CENTERED_DEG  → cable is straight ahead
      - Tension is between LOW_TENSION and HIGH_TENSION → cable is neither slack nor over-taut

    When active: publishes CRUISE_BOOST to 'cruise_boost' topic.
    When inactive: publishes 0.0 so ThrusterNode returns to reactive-only control.

    ThrusterNode sums this with the depth-follow boost from follower.py.
    """

    def __init__(self):
        super().__init__('cruise_control')

        self._angle   = 0.0
        self._tension = 0.0

        self.create_subscription(Float32, 'encoder_angle', self._on_angle,   10)
        self.create_subscription(Float32, 'tension',       self._on_tension, 10)

        self._pub = self.create_publisher(Float32, 'cruise_boost', 10)

        self.create_timer(0.1, self._update)   # 10 Hz

        self.get_logger().info(
            f'Cruise control ready. '
            f'Active when |angle|<{DEAD_ZONE_DEG}° (steering dead zone) '
            f'and {LOW_TENSION}<tension<{HIGH_TENSION} kg. '
            f'Boost={CRUISE_BOOST} µs.'
        )

    def _on_angle(self, msg: Float32):
        self._angle = float(msg.data)

    def _on_tension(self, msg: Float32):
        self._tension = float(msg.data)

    def _update(self):
        cable_straight  = abs(self._angle) < DEAD_ZONE_DEG
        tension_nominal = LOW_TENSION < self._tension < HIGH_TENSION

        boost = float(CRUISE_BOOST) if (cable_straight and tension_nominal) else 0.0

        self._pub.publish(Float32(data=boost))
        self.get_logger().debug(
            f'angle={self._angle:.1f}° tension={self._tension:.1f} → cruise_boost={boost}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CruiseControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
