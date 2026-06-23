#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray

# Physical spool constants (identical to follower.py and display_new.py)
_SPOOL_CORE_RADIUS = 0.095    # metres
_CABLE_DIAMETER    = 0.0076   # metres
_SPINS_PER_LAYER   = 23.125

# Fraction applied to the Pythagoras horizontal estimate to correct for cable sag.
# The cable arcs between USV and ROV, so the true horizontal endpoint distance is
# shorter than sqrt(L²-d²). 0.85 is a conservative default; tune empirically.
_DEFAULT_SAG_FACTOR = 0.85


def _cable_length_from_spins(spins: float) -> float:
    """Return deployed cable length in metres for a given accumulated spin count."""
    spins = max(0.0, spins)
    n = int(spins / _SPINS_PER_LAYER)
    partial = spins % _SPINS_PER_LAYER

    length = 2.0 * partial * math.pi * (
        _SPOOL_CORE_RADIUS + _CABLE_DIAMETER / 2.0 + n * _CABLE_DIAMETER
    )
    for i in range(n):
        length += 2.0 * _SPINS_PER_LAYER * math.pi * (
            _SPOOL_CORE_RADIUS + _CABLE_DIAMETER / 2.0 + i * _CABLE_DIAMETER
        )
    return length


class ROVPositionNode(Node):
    """
    Estimates the ROV's absolute GPS position from four inputs:

      - USV GPS           → 'gps' topic  [fix, lat, lon, alt]
      - Winch stepper RPM → 'stepper' topic  (integrated → cable arc-length L)
      - ROV depth         → 'subm' topic  (positive metres, published by ArduSub)
      - ROV heading (yaw) → MAVLink ATTITUDE message from the BlueROV2

    The cable exits the centre-back of the ROV, so the ROV's magnetic heading
    equals the bearing from USV to ROV. Combined with the horizontal distance
    derived from Pythagoras on (L, depth), this gives an N/E offset that is
    added to the USV GPS to produce absolute ROV lat/lon.

    Publishes to 'rov_position': Float32MultiArray([lat_deg, lon_deg, depth_m])
    """

    TIMER_DT = 0.2  # seconds — integration step and publish rate (5 Hz)

    def __init__(self, mav_connection=None, sag_factor: float = _DEFAULT_SAG_FACTOR):
        super().__init__('rov_position')

        self._sag_factor = sag_factor

        # USV state
        self._usv_lat: float | None = None
        self._usv_lon: float | None = None

        # Cable integration (same pattern as follower.py)
        self._stepper_rpm: float = 0.0
        self._spins: float = 0.0

        # ROV state
        self._rov_depth: float = 0.0        # metres, positive downward
        self._rov_heading_rad: float = 0.0  # magnetic bearing from MAVLink, NED radians

        # Subscriptions
        self.create_subscription(Float64MultiArray, 'gps',     self._on_gps,     10)
        self.create_subscription(Float32,           'stepper', self._on_stepper, 10)
        self.create_subscription(Float32,           'subm',    self._on_depth,   10)

        # Publisher
        self._pub = self.create_publisher(Float32MultiArray, 'rov_position', 10)

        # Compute + publish timer
        self.create_timer(self.TIMER_DT, self._compute_and_publish)

        # Optional MAVLink listener for ROV heading
        if mav_connection is not None:
            self._mav = mav_connection
            t = threading.Thread(target=self._mavlink_listener, daemon=True)
            t.start()
        else:
            self._mav = None
            self.get_logger().warn(
                'ROVPositionNode: no MAVLink connection provided — '
                'ROV heading fixed at 0.0 rad (North). Pass mav_connection for real bearing.'
            )

        self.get_logger().info('ROV position node started.')

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                        #
    # ------------------------------------------------------------------ #

    def _on_gps(self, msg: Float64MultiArray):
        fix, lat, lon, _alt = msg.data
        if fix >= 3 and lat != 0.0:
            self._usv_lat = float(lat)
            self._usv_lon = float(lon)

    def _on_stepper(self, msg: Float32):
        self._stepper_rpm = float(msg.data)

    def _on_depth(self, msg: Float32):
        # 'subm' publishes positive depth; follower.py negates it internally.
        # We keep it positive (downward) here.
        self._rov_depth = abs(float(msg.data))

    # ------------------------------------------------------------------ #
    # MAVLink background thread                                            #
    # ------------------------------------------------------------------ #

    def _mavlink_listener(self):
        """Continuously read ATTITUDE messages from the BlueROV2 to update heading."""
        while rclpy.ok():
            try:
                msg = self._mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
                if msg:
                    # yaw: NED radians, 0 = North, positive clockwise
                    self._rov_heading_rad = float(msg.yaw)
            except Exception as exc:
                self.get_logger().warn(f'MAVLink ATTITUDE read error: {exc}')

    # ------------------------------------------------------------------ #
    # Position computation                                                 #
    # ------------------------------------------------------------------ #

    def _compute_and_publish(self):
        if self._usv_lat is None:
            return  # Wait until we have a GPS fix

        # Integrate cable spins (same fixed-dt method as follower.py)
        self._spins += (self._stepper_rpm / 60.0) * self.TIMER_DT
        self._spins = max(0.0, self._spins)

        L = _cable_length_from_spins(self._spins)
        d = self._rov_depth

        # Horizontal distance from USV to ROV (corrected for cable sag)
        if L <= d:
            # Inconsistent data: cable shorter than depth. ROV is directly below.
            horizontal_dist = 0.0
        else:
            horizontal_dist = math.sqrt(L ** 2 - d ** 2) * self._sag_factor

        # Cable bearing = ROV magnetic heading (cable exits centre-back of ROV)
        bearing = self._rov_heading_rad  # NED radians, 0 = North

        north_m = horizontal_dist * math.cos(bearing)
        east_m  = horizontal_dist * math.sin(bearing)

        # Convert metre offset to lat/lon delta (same formula as imu.py:79-81)
        delta_lat = north_m / 111000.0
        delta_lon = east_m  / (111000.0 * math.cos(math.radians(self._usv_lat)))

        rov_lat = self._usv_lat + delta_lat
        rov_lon = self._usv_lon + delta_lon

        out = Float32MultiArray()
        out.data = [float(rov_lat), float(rov_lon), float(d)]
        self._pub.publish(out)

        self.get_logger().debug(
            f'ROV pos: lat={rov_lat:.7f} lon={rov_lon:.7f} depth={d:.2f}m '
            f'(L={L:.2f}m H={horizontal_dist:.2f}m bearing={math.degrees(bearing):.1f}°)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ROVPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
