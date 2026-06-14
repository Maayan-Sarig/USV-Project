#!/usr/bin/env python3
"""
Station Keeping — active GPS position hold for the USV.

Active when usv_mode topic == 'STATION_KEEPING'.

Home position comes from the 'home_position' topic published by usv_sensor_bridge
(sourced from QGC via MAVLink HOME_POSITION). Falls back to the first valid GPS
fix if no home_position arrives within the first 10 s.

Navigation (no compass required):
  desired_bearing = atan2( (home_lon − cur_lon) × cos(lat),  home_lat − cur_lat )
  actual_bearing  = GPS track heading from consecutive GPS fixes
  steering_error  = wrap_180(desired_bearing − actual_bearing)
  sk_angle        = clip(steering_error, −75, 75)
  sk_boost        = clip(distance_m × 5.0, 0, MAX_BOOST_US)

When within HOLD_RADIUS_M of home: publishes 0.0, 0.0 (winch holds the rest).
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

# ── Tunable parameters ────────────────────────────────────────────────────────
HOLD_RADIUS_M  = 3.0    # metres — dead zone, no correction inside this radius
MAX_BOOST_US   = 60.0   # µs — maximum forward boost sent to T200s
BOOST_PER_M    = 5.0    # µs of forward boost per metre of position error
MIN_SPEED_MS   = 0.3    # m/s — below this speed GPS track heading is unreliable
METERS_PER_DEG = 111_000.0  # approximate metres per degree of latitude


def _wrap_180(deg: float) -> float:
    """Wrap angle to (−180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def _bearing(from_lat, from_lon, to_lat, to_lon):
    """Return North-up bearing in degrees from (from) to (to)."""
    cos_lat = math.cos(math.radians(from_lat))
    dy = (to_lat - from_lat) * METERS_PER_DEG
    dx = (to_lon - from_lon) * METERS_PER_DEG * cos_lat
    return math.degrees(math.atan2(dx, dy))  # atan2(East, North) = bearing from North


def _distance_m(lat1, lon1, lat2, lon2):
    """Return approximate distance in metres between two GPS coordinates."""
    cos_lat = math.cos(math.radians(lat1))
    dy = (lat2 - lat1) * METERS_PER_DEG
    dx = (lon2 - lon1) * METERS_PER_DEG * cos_lat
    return math.sqrt(dx * dx + dy * dy)


class StationKeepingNode(Node):
    def __init__(self):
        super().__init__('station_keeping')

        self._mode       = 'STATION_KEEPING'
        self._home_lat   = None
        self._home_lon   = None
        self._cur_lat    = None
        self._cur_lon    = None
        self._prev_lat   = None
        self._prev_lon   = None
        self._prev_time  = None
        self._track_hdg  = None   # GPS track heading in degrees, None until first movement

        # Subscriptions
        self.create_subscription(Float32MultiArray, 'gps',           self._on_gps,  10)
        self.create_subscription(Float32MultiArray, 'home_position', self._on_home, 10)
        self.create_subscription(String,            'usv_mode',      self._on_mode, 10)

        # Outputs → t200.py
        self._angle_pub = self.create_publisher(Float32, 'sk_angle', 10)
        self._boost_pub = self.create_publisher(Float32, 'sk_boost', 10)

        self.create_timer(0.2, self._update)  # 5 Hz
        self.get_logger().info(
            f'Station keeping ready. Hold radius={HOLD_RADIUS_M} m, '
            f'max boost={MAX_BOOST_US} µs.'
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_home(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self._home_lat = float(msg.data[0])
            self._home_lon = float(msg.data[1])
            self.get_logger().info(
                f'Home position received: {self._home_lat:.6f}, {self._home_lon:.6f}'
            )

    def _on_gps(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        fix = msg.data[0]
        if fix < 3:
            return
        lat, lon = float(msg.data[1]), float(msg.data[2])

        # Auto-set home from first valid fix if QGC home hasn't arrived yet
        if self._home_lat is None:
            self._home_lat = lat
            self._home_lon = lon
            self.get_logger().info(
                f'Home auto-set from first GPS fix: {lat:.6f}, {lon:.6f}'
            )

        # Update GPS track heading
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_lat is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                dist = _distance_m(self._prev_lat, self._prev_lon, lat, lon)
                speed = dist / dt
                if speed >= MIN_SPEED_MS:
                    self._track_hdg = _bearing(self._prev_lat, self._prev_lon, lat, lon)

        self._prev_lat  = self._cur_lat
        self._prev_lon  = self._cur_lon
        self._prev_time = now
        self._cur_lat   = lat
        self._cur_lon   = lon

    def _on_mode(self, msg: String):
        self._mode = msg.data

    # ── Control loop ──────────────────────────────────────────────────────────

    def _update(self):
        if self._mode != 'STATION_KEEPING':
            self._publish(0.0, 0.0)
            return

        if None in (self._home_lat, self._home_lon, self._cur_lat, self._cur_lon):
            return  # waiting for first GPS fix

        dist = _distance_m(self._cur_lat, self._cur_lon, self._home_lat, self._home_lon)

        if dist < HOLD_RADIUS_M:
            self._publish(0.0, 0.0)
            return

        desired_bearing = _bearing(self._cur_lat, self._cur_lon, self._home_lat, self._home_lon)

        if self._track_hdg is None:
            # No movement yet — cannot compute steering error. Apply small forward nudge.
            self._publish(0.0, min(MAX_BOOST_US, dist * BOOST_PER_M))
            return

        steering_error = _wrap_180(desired_bearing - self._track_hdg)
        sk_angle = max(-75.0, min(75.0, steering_error))
        sk_boost = min(MAX_BOOST_US, dist * BOOST_PER_M)

        self.get_logger().debug(
            f'SK dist={dist:.1f}m desired={desired_bearing:.1f}° '
            f'actual={self._track_hdg:.1f}° error={steering_error:.1f}° '
            f'angle_cmd={sk_angle:.1f}° boost={sk_boost:.0f}µs'
        )
        self._publish(sk_angle, sk_boost)

    def _publish(self, angle: float, boost: float):
        self._angle_pub.publish(Float32(data=angle))
        self._boost_pub.publish(Float32(data=boost))

    # ── External API (called by RTLNode to reuse GPS navigation logic) ────────

    def navigate_to(self, target_lat: float, target_lon: float):
        """Override home to navigate toward an arbitrary GPS coordinate."""
        self._home_lat = target_lat
        self._home_lon = target_lon


def main(args=None):
    rclpy.init(args=args)
    node = StationKeepingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
