#!/usr/bin/env python3
"""
ROV Mirror — replicate the operator's ROV joystick in the horizontal plane on the USV.

Active when usv_mode topic == 'ROV_MIRROR' AND the ROV's own ArduSub flight mode
(published on 'rov_flight_mode' by rtl.py's existing MAVLink listener thread)
== 'MANUAL'. Either condition failing — or no rov_manual_control seen recently —
publishes neutral (0.0, 0.0), so the USV always fails safe back to idle.

This node has no MAVLink connection of its own. 'rov_flight_mode' and
'rov_manual_control' are republished by rtl.py's _heartbeat_monitor thread,
which already reads the shared MAVLink connection for failsafe monitoring —
piggybacking here avoids adding another competing reader on that connection.

Only x (forward/back) and r (turn) are used. y (lateral) is ignored — the USV
has no strafing capability. z (vertical/depth) is ignored by design.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

# ── Tunable parameters ────────────────────────────────────────────────────────
MAX_ANGLE_DEG       = 75.0   # degrees — full stick deflection on 'r' -> hard turn (matches t200.py)
MAX_MIRROR_BOOST_US = 100.0  # µs — full stick deflection on 'x' -> max forward/reverse boost
STALE_TIMEOUT_S     = 0.75   # s — no rov_manual_control within this window -> treat as neutral
ROV_MANUAL_MODE_NAME = 'MANUAL'


class ROVMirrorNode(Node):
    def __init__(self):
        super().__init__('rov_mirror')

        self._usv_mode        = None
        self._rov_mode_name   = None
        self._last_x          = 0.0
        self._last_r          = 0.0
        self._last_manual_ts  = 0.0

        # Subscriptions
        self.create_subscription(String,            'usv_mode',           self._on_usv_mode,  10)
        self.create_subscription(String,            'rov_flight_mode',    self._on_rov_mode,  10)
        self.create_subscription(Float32MultiArray,  'rov_manual_control', self._on_manual,    10)

        # Outputs -> t200.py
        self._angle_pub = self.create_publisher(Float32, 'mirror_angle', 10)
        self._boost_pub = self.create_publisher(Float32, 'mirror_boost', 10)

        self.create_timer(0.1, self._update)  # 10 Hz

        self.get_logger().info('ROV mirror node ready (pure-ROS, no MAVLink connection).')

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_usv_mode(self, msg: String):
        self._usv_mode = msg.data

    def _on_rov_mode(self, msg: String):
        self._rov_mode_name = msg.data

    def _on_manual(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self._last_x = float(msg.data[0])
        self._last_r = float(msg.data[1])
        self._last_manual_ts = time.time()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _update(self):
        stale = (time.time() - self._last_manual_ts) > STALE_TIMEOUT_S
        if self._usv_mode != 'ROV_MIRROR' or self._rov_mode_name != ROV_MANUAL_MODE_NAME or stale:
            self._publish(0.0, 0.0)
            return

        mirror_angle = (self._last_r / 1000.0) * MAX_ANGLE_DEG
        mirror_boost = (self._last_x / 1000.0) * MAX_MIRROR_BOOST_US

        self.get_logger().debug(
            f'Mirror: x={self._last_x:.0f} r={self._last_r:.0f} '
            f'-> angle={mirror_angle:.1f}° boost={mirror_boost:.0f}µs'
        )
        self._publish(mirror_angle, mirror_boost)

    def _publish(self, angle: float, boost: float):
        self._angle_pub.publish(Float32(data=angle))
        self._boost_pub.publish(Float32(data=boost))


def main(args=None):
    rclpy.init(args=args)
    node = ROVMirrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
