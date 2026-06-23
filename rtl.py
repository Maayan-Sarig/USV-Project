#!/usr/bin/env python3
"""
RTL (Return to Launch) — automated failsafe node.

Monitors three independent trigger conditions:
  1. Cable tension > 135 kg (over-tension — tether about to snap)
  2. MAVLink heartbeat loss > 30 s (comms failure to BlueROV2)
  3. ArduSub water leak detected  (NAMED_VALUE_FLOAT name='Leak' value>0.5)

RTL sequence (non-interruptible once started):
  Step 1  Announce:  publish rtl_trigger reason + set usv_mode = 'RTL'
  Step 2  Surface ROV:  set ArduSub mode to SURFACE
  Step 3  Wait:  USV stays still until tension < SURFACE_TENSION_KG (cable slack)
                 Timeout = SURFACE_WAIT_TIMEOUT_S
  Step 4  Disarm ROV via MAVLink
  Step 5  Navigate USV to home GPS (GPS-bearing-based, no compass needed)
  Step 6  Arrived: stop thrusters, switch to STATION_KEEPING

Navigation (same algorithm as station_keeping.py):
  desired_bearing = atan2( (home_lon−cur_lon)×cos(lat), home_lat−cur_lat )
  actual_bearing  = GPS track heading (consecutive USV GPS fixes)
  steering_error  = wrap_180(desired_bearing − actual_bearing)
  sk_angle = clip(steering_error, −75, 75)
  sk_boost = clip(dist×BOOST_PER_M, MIN_RTL_BOOST, MAX_BOOST_US)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String

from pymavlink import mavutil

# ── Trigger thresholds ────────────────────────────────────────────────────────
MAX_TENSION_KG           = 135.0   # kg — over-tension trigger
COMMS_TIMEOUT_S          = 30.0    # s  — heartbeat loss trigger
SURFACE_TENSION_KG       = 5.0     # kg — tension below this → ROV is at surface
SURFACE_WAIT_TIMEOUT_S   = 120.0   # s  — max time waiting for ROV to surface

# ── Navigation parameters ─────────────────────────────────────────────────────
HOME_RADIUS_M  = 10.0   # metres — RTL arrival threshold
MAX_BOOST_US   = 60.0   # µs
MIN_RTL_BOOST  = 15.0   # µs — minimum boost so USV actually moves
BOOST_PER_M    = 5.0    # µs per metre of distance
MIN_SPEED_MS   = 0.3    # m/s below which GPS track heading is frozen
METERS_PER_DEG = 111_000.0


def _wrap_180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _bearing(from_lat, from_lon, to_lat, to_lon):
    cos_lat = math.cos(math.radians(from_lat))
    dy = (to_lat - from_lat) * METERS_PER_DEG
    dx = (to_lon - from_lon) * METERS_PER_DEG * cos_lat
    return math.degrees(math.atan2(dx, dy))


def _distance_m(lat1, lon1, lat2, lon2):
    cos_lat = math.cos(math.radians(lat1))
    dy = (lat2 - lat1) * METERS_PER_DEG
    dx = (lon2 - lon1) * METERS_PER_DEG * cos_lat
    return math.sqrt(dx * dx + dy * dy)


class RTLNode(Node):
    def __init__(self, mav_connection=None, mav_lock=None):
        super().__init__('rtl_node')

        self._mav            = mav_connection
        self._mav_lock       = mav_lock or threading.Lock()
        self._rtl_active     = False
        self._tension        = 0.0
        self._home_lat       = None
        self._home_lon       = None
        self._cur_lat        = None
        self._cur_lon        = None
        self._prev_lat       = None
        self._prev_lon       = None
        self._prev_time      = None
        self._track_hdg      = None
        self._last_heartbeat = time.time()

        # Subscriptions
        self.create_subscription(Float32,            'tension',       self._on_tension, 10)
        self.create_subscription(Float64MultiArray,  'gps',           self._on_gps,     10)
        self.create_subscription(Float32MultiArray,  'home_position', self._on_home,    10)

        # Outputs
        self._mode_pub    = self.create_publisher(String,  'usv_mode',    10)
        self._trigger_pub = self.create_publisher(String,  'rtl_trigger', 10)
        self._angle_pub   = self.create_publisher(Float32, 'sk_angle',    10)
        self._boost_pub   = self.create_publisher(Float32, 'sk_boost',    10)
        self._rov_mode_pub    = self.create_publisher(String,            'rov_flight_mode',   10)
        self._manual_ctrl_pub = self.create_publisher(Float32MultiArray, 'rov_manual_control', 10)

        # Tension + leak monitor at 5 Hz
        self.create_timer(0.2, self._check_triggers)

        # Heartbeat monitor thread (non-blocking MAVLink recv)
        if self._mav is not None:
            t = threading.Thread(target=self._heartbeat_monitor, daemon=True)
            t.start()

        self.get_logger().info(
            f'RTL node ready. Triggers: tension>{MAX_TENSION_KG} kg, '
            f'comms>{COMMS_TIMEOUT_S} s, water leak.'
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_tension(self, msg: Float32):
        self._tension = float(msg.data)

    def _on_home(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self._home_lat = float(msg.data[0])
            self._home_lon = float(msg.data[1])

    def _on_gps(self, msg: Float64MultiArray):
        if len(msg.data) < 3 or msg.data[0] < 3:
            return
        lat, lon = float(msg.data[1]), float(msg.data[2])
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._prev_lat is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                dist = _distance_m(self._prev_lat, self._prev_lon, lat, lon)
                if dist / dt >= MIN_SPEED_MS:
                    self._track_hdg = _bearing(self._prev_lat, self._prev_lon, lat, lon)

        self._prev_lat  = self._cur_lat
        self._prev_lon  = self._cur_lon
        self._prev_time = now
        self._cur_lat   = lat
        self._cur_lon   = lon

    # ── Trigger monitoring ────────────────────────────────────────────────────

    def _check_triggers(self):
        if self._rtl_active:
            return
        if self._tension > MAX_TENSION_KG:
            self._trigger_rtl('tension')

    def _heartbeat_monitor(self):
        """Background thread: polls MAVLink for HEARTBEAT, water leak, and
        operator MANUAL_CONTROL messages. This is the only thread in the
        process reading the shared MAVLink connection's HEARTBEAT/MANUAL_CONTROL
        traffic — rov_mirror.py rides on these two republished ROS topics
        instead of opening its own competing MAVLink listener thread."""
        while rclpy.ok():
            try:
                with self._mav_lock:
                    msg = self._mav.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    pass
                elif msg.get_type() == 'HEARTBEAT':
                    self._last_heartbeat = time.time()
                    if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                        self._rov_mode_pub.publish(String(data=mavutil.mode_string_v10(msg)))
                elif msg.get_type() == 'NAMED_VALUE_FLOAT':
                    name = getattr(msg, 'name', b'').strip(b'\x00').decode('ascii', errors='ignore')
                    if name == 'Leak' and float(msg.value) > 0.5 and not self._rtl_active:
                        self._trigger_rtl('leak')
                elif msg.get_type() == 'MANUAL_CONTROL':
                    if msg.target == self._mav.target_system:
                        self._manual_ctrl_pub.publish(Float32MultiArray(data=[float(msg.x), float(msg.r)]))
            except Exception:
                time.sleep(0.5)

            if not self._rtl_active and time.time() - self._last_heartbeat > COMMS_TIMEOUT_S:
                self._trigger_rtl('comms_loss')

    # ── RTL execution ─────────────────────────────────────────────────────────

    def _trigger_rtl(self, reason: str):
        if self._rtl_active:
            return
        self._rtl_active = True
        self.get_logger().error(f'RTL TRIGGERED: {reason}')
        threading.Thread(target=self._rtl_sequence, args=(reason,), daemon=True).start()

    def _rtl_sequence(self, reason: str):
        # ── Step 1: Announce ─────────────────────────────────────────────────
        self._trigger_pub.publish(String(data=reason))
        self._mode_pub.publish(String(data='RTL'))
        self._publish_cmd(0.0, 0.0)   # thrusters idle while ROV surfaces

        # ── Step 2: Command ROV to surface ───────────────────────────────────
        if self._mav is not None:
            try:
                from blue_rov2_terminal_control import set_mode
                set_mode(self._mav, 'SURFACE')
                self.get_logger().info('RTL: ROV commanded to SURFACE mode')
            except Exception as e:
                self.get_logger().warn(f'RTL: set_mode SURFACE failed: {e}')

        # ── Step 3: Wait for ROV to reach surface (tension drops) ────────────
        self.get_logger().info(
            f'RTL: waiting for tension < {SURFACE_TENSION_KG} kg '
            f'(timeout {SURFACE_WAIT_TIMEOUT_S} s)…'
        )
        deadline = time.time() + SURFACE_WAIT_TIMEOUT_S
        while time.time() < deadline:
            if self._tension < SURFACE_TENSION_KG:
                self.get_logger().info(
                    f'RTL: tension={self._tension:.1f} kg — ROV at surface.'
                )
                break
            time.sleep(1.0)
        else:
            self.get_logger().warn(
                f'RTL: surface wait timed out after {SURFACE_WAIT_TIMEOUT_S} s. '
                'Proceeding anyway.'
            )

        # ── Step 4: Disarm ROV ────────────────────────────────────────────────
        if self._mav is not None:
            try:
                from blue_rov2_terminal_control import arm
                arm(self._mav, False)
                self.get_logger().info('RTL: ROV disarmed.')
            except Exception as e:
                self.get_logger().warn(f'RTL: disarm failed: {e}')

        # ── Step 5: Navigate USV to home ─────────────────────────────────────
        if self._home_lat is None or self._home_lon is None:
            self.get_logger().error(
                'RTL: no home position available — cannot navigate home. '
                'Switching to STATION_KEEPING.'
            )
        else:
            self.get_logger().info(
                f'RTL: navigating to home {self._home_lat:.6f}, {self._home_lon:.6f}'
            )
            self._navigate_home()

        # ── Step 6: Arrived ───────────────────────────────────────────────────
        self._publish_cmd(0.0, 0.0)
        self._mode_pub.publish(String(data='STATION_KEEPING'))
        self._rtl_active = False
        self.get_logger().info('RTL: complete — switched to STATION_KEEPING.')

    def _navigate_home(self):
        """Drive USV toward home using GPS bearing until within HOME_RADIUS_M."""
        while rclpy.ok():
            if self._cur_lat is None or self._home_lat is None:
                time.sleep(0.5)
                continue

            dist = _distance_m(self._cur_lat, self._cur_lon, self._home_lat, self._home_lon)
            if dist < HOME_RADIUS_M:
                self.get_logger().info(f'RTL: arrived at home (dist={dist:.1f} m).')
                break

            desired = _bearing(self._cur_lat, self._cur_lon, self._home_lat, self._home_lon)

            if self._track_hdg is not None:
                error    = _wrap_180(desired - self._track_hdg)
                sk_angle = max(-75.0, min(75.0, error))
            else:
                sk_angle = 0.0   # no heading yet — go straight

            sk_boost = max(MIN_RTL_BOOST, min(MAX_BOOST_US, dist * BOOST_PER_M))

            self._publish_cmd(sk_angle, sk_boost)
            self.get_logger().debug(
                f'RTL nav: dist={dist:.1f}m desired={desired:.1f}° '
                f'actual={self._track_hdg}° angle={sk_angle:.1f}° boost={sk_boost:.0f}µs'
            )
            time.sleep(0.2)   # 5 Hz

    def _publish_cmd(self, angle: float, boost: float):
        self._angle_pub.publish(Float32(data=angle))
        self._boost_pub.publish(Float32(data=boost))


def main(args=None):
    rclpy.init(args=args)
    node = RTLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
