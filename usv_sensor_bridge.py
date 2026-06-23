import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String
import datetime
import socket
import json
import time
import threading
from typing import Optional

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except Exception:
    MAVLINK_AVAILABLE = False

_GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)


def _gps_week_and_tow_ms():
    """Return (gps_week, time_of_week_ms) for the current instant, for GPS_INPUT's
    time_week/time_week_ms fields. Ignores the ~18s UTC/GPS leap-second offset —
    fine for EKF fusion, which doesn't need bit-exact GPS time."""
    delta = datetime.datetime.now(datetime.timezone.utc) - _GPS_EPOCH
    gps_week = int(delta.days // 7)
    seconds_into_week = delta.total_seconds() - gps_week * 604800
    return gps_week, int(seconds_into_week * 1000)


class USVSensorBridge(Node):
    def __init__(self, udp_host='192.168.1.100', udp_port=14550, send_mavlink=True, mav_connection=None):
        super().__init__('usv_sensor_bridge')

        self.udp_host = udp_host
        self.udp_port = udp_port
        self.send_mavlink = send_mavlink and MAVLINK_AVAILABLE
        self.mavlink_master = mav_connection

        self.latest = {
            'gps': None,          # [fix, lat, lon, alt]
            'battery': None,      # {'voltage':..., 'percent':...}
            'tension': None,      # float
            'encoder': None,      # degrees float
            'motors': None,       # list or tuple
            'rov_data': None,     # passthrough raw data
            'rov_position': None, # [lat_deg, lon_deg, depth_m]
        }

        self._home_origin_sent = False  # Flag: SET_GPS_GLOBAL_ORIGIN sent on first valid fix
        self._gps_type_confirmed = False  # Flag: ArduSub confirmed GPS_TYPE=14 (MAVLink)

        # Subscribers (match what's used in the repo)
        self.create_subscription(Float32MultiArray, 'gps', self.cb_gps, 10)
        self.create_subscription(Float32, 'tension', self.cb_tension, 10)
        self.create_subscription(Float32, 'encoder_angle', self.cb_encoder, 10)
        self.create_subscription(Float32MultiArray, 't200_speed', self.cb_motors, 10)
        # battery topic may not exist; listen on 'battery' if present
        self.create_subscription(Float32MultiArray, 'battery', self.cb_battery, 10)
        self.create_subscription(Float32MultiArray, 'rov_position', self.cb_rov_position, 10)

        # Publish aggregated state as JSON on ROS topic
        self.pub_state = self.create_publisher(String, 'usv/state', 10)

        # Publish QGC home position so station_keeping and rtl nodes can use it
        self._home_pub = self.create_publisher(Float32MultiArray, 'home_position', 10)
        self._home_requested = False

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # MAVLink setup — two virtual vehicles so QGC shows both on the map:
        #   sysid=2  MAV_TYPE_SURFACE_BOAT (7)  → USV  (boat icon)
        #   sysid=3  MAV_TYPE_SUBMARINE   (12) → calculated ROV position (sub icon)
        if self.send_mavlink:
            self.get_logger().info(
                'pymavlink available: sending MAVLink as sysid=2 (USV) and sysid=3 (calc ROV)'
            )
            self.mav_usv = mavutil.mavlink.MAVLink(None)
            self.mav_usv.srcSystem = 2
            self.mav_usv.srcComponent = 1

            self.mav_rov_calc = mavutil.mavlink.MAVLink(None)
            self.mav_rov_calc.srcSystem = 3
            self.mav_rov_calc.srcComponent = 1
        elif not MAVLINK_AVAILABLE and send_mavlink:
            self.get_logger().warn('Requested MAVLink send but pymavlink not available — falling back to JSON')

        # Timer to send snapshot at 5 Hz
        self.timer = self.create_timer(0.2, self.send_snapshot)

        # Heartbeat timer at 1 Hz (MAVLink standard requirement for QGC to recognise vehicles)
        if self.send_mavlink:
            self.create_timer(1.0, self._send_heartbeats)

        # Home position: request once at startup, re-request every 60 s
        if self.send_mavlink:
            self.create_timer(60.0, self._request_home_position)

        # GPS_TYPE must be 14 ("MAVLink") on the ROV's autopilot or it will
        # silently ignore every GPS_INPUT message we inject below — set it
        # once now, then keep retrying every 10s until ArduSub confirms it
        # (PARAM_SET over UDP can be dropped, and there's no harm re-sending).
        if self.mavlink_master is not None:
            self._ensure_gps_type_mavlink()
            self.create_timer(10.0, self._ensure_gps_type_mavlink)

        # --- Video forwarder configuration (unprocessed UDP video stream) ---
        # Parameters can be set via ROS2 CLI, e.g.:
        #  ros2 run ... --ros-args -p video_forward.enabled:=True -p video_forward.listen_port:=5600 -p video_forward.rf_host:=192.168.137.1 -p video_forward.rf_port:=5600
        self.declare_parameter('video_forward.enabled', False)
        self.declare_parameter('video_forward.listen_port', 5600)
        self.declare_parameter('video_forward.rf_host', '192.168.137.1')
        self.declare_parameter('video_forward.rf_port', 5600)

        self.video_enabled = bool(self.get_parameter('video_forward.enabled').value)
        self.video_listen_port = int(self.get_parameter('video_forward.listen_port').value)
        self.rf_host = str(self.get_parameter('video_forward.rf_host').value)
        self.rf_port = int(self.get_parameter('video_forward.rf_port').value)

        self._video_thread: Optional[threading.Thread] = None
        self._video_stop = threading.Event()
        self._video_send_sock: Optional[socket.socket] = None

        if self.video_enabled:
            self.get_logger().info(f"Starting video forwarder: 0.0.0.0:{self.video_listen_port} -> {self.rf_host}:{self.rf_port}")
            self._video_thread = threading.Thread(target=self._video_forward_loop, daemon=True)
            self._video_thread.start()

    def cb_gps(self, msg: Float32MultiArray):
        try:
            self.latest['gps'] = list(msg.data)
            # Inject GPS data to ROV autopilot via MAVLink GPS_INPUT
            if self.mavlink_master is not None and len(msg.data) >= 4:
                fix_type = int(msg.data[0])
                lat = msg.data[1]
                lon = msg.data[2]
                alt = msg.data[3]
                # Only inject if we have a valid 2D or 3D fix
                if fix_type >= 2:
                    # On first valid fix, send SET_GPS_GLOBAL_ORIGIN to establish home position
                    if not self._home_origin_sent:
                        self.set_gps_global_origin(lat, lon, alt)
                        self._home_origin_sent = True
                    self.inject_gps_to_rov(lat, lon, alt, fix_type, num_sats=15)
        except Exception as e:
            self.get_logger().warn(f'GPS callback error: {e}')

    def cb_tension(self, msg: Float32):
        self.latest['tension'] = float(msg.data)

    def cb_encoder(self, msg: Float32):
        self.latest['encoder'] = float(msg.data)

    def cb_motors(self, msg: Float32MultiArray):
        try:
            self.latest['motors'] = list(msg.data)
        except Exception:
            pass

    def cb_battery(self, msg: Float32MultiArray):
        try:
            # expect [voltage, percent] or similar
            data = list(msg.data)
            if len(data) >= 2:
                self.latest['battery'] = {'voltage': float(data[0]), 'percent': float(data[1])}
            else:
                self.latest['battery'] = {'voltage': float(data[0])}
        except Exception:
            pass

    def cb_rov_position(self, msg: Float32MultiArray):
        try:
            self.latest['rov_position'] = list(msg.data)  # [lat_deg, lon_deg, depth_m]
        except Exception:
            pass

    def _ensure_gps_type_mavlink(self):
        """
        Set ArduSub's GPS_TYPE parameter to 14 ("MAVLink") so it accepts our
        injected GPS_INPUT messages instead of expecting a physical GPS
        receiver wired to a serial port — without this, GPS_INPUT is just
        silently ignored no matter how correct the data is. Keeps retrying
        (via the 10s timer set up in __init__) until ArduSub confirms it via
        PARAM_VALUE, since PARAM_SET over UDP can be dropped.
        """
        if self.mavlink_master is None or self._gps_type_confirmed:
            return
        GPS_TYPE_TARGET = 14
        try:
            self.mavlink_master.mav.param_set_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                b'GPS_TYPE',
                float(GPS_TYPE_TARGET),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            msg = self.mavlink_master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg is not None and msg.param_id.strip('\x00') == 'GPS_TYPE':
                if int(msg.param_value) == GPS_TYPE_TARGET:
                    self._gps_type_confirmed = True
                    self.get_logger().info(
                        'GPS_TYPE confirmed = 14 (MAVLink). If this was just set for the '
                        'first time, ArduSub needs a reboot for it to take effect.'
                    )
                else:
                    self.get_logger().warn(
                        f'GPS_TYPE is {int(msg.param_value)}, not 14 — sent PARAM_SET, awaiting ack'
                    )
        except Exception as e:
            self.get_logger().warn(f'GPS_TYPE PARAM_SET failed: {e}')

    def inject_gps_to_rov(self, lat, lon, alt, fix_type=3, num_sats=15):
        """
        Injects USV GPS data into the ROV autopilot using MAVLink GPS_INPUT.
        
        :param lat: Latitude in decimal degrees
        :param lon: Longitude in decimal degrees
        :param alt: Altitude/Height in meters (MSL)
        :param fix_type: 3 = 3D Fix (required for AUTO mode); 2 = 2D Fix
        :param num_sats: Number of visible satellites to display in QGC
        """
        try:
            if self.mavlink_master is None:
                return
            
            # MAVLink expects time since boot in microseconds
            boot_time_us = int(time.time() * 1e6)
            
            # Convert decimal degrees to integer format expected by MAVLink (deg * 1e7)
            lat_int = int(lat * 1e7)
            lon_int = int(lon * 1e7)

            time_week, time_week_ms = _gps_week_and_tow_ms()

            # Send the GPS_INPUT message to the ROV
            self.mavlink_master.mav.gps_input_send(
                boot_time_us,           # time_usec
                0,                      # gps_id (0 for primary virtual GPS instance)
                0,                      # ignore_flags (0 = all data valid)
                time_week_ms,           # time_week_ms: ms since start of current GPS week
                time_week,              # time_week: weeks since the GPS epoch (1980-01-06)
                int(fix_type),          # fix_type: 3 = 3D Fix, 2 = 2D Fix
                lat_int,                # lat (deg * 1e7)
                lon_int,                # lon (deg * 1e7)
                float(alt),             # alt (meters MSL)
                1.0,                    # hdop: Horizontal dilution of precision (1.0 is excellent)
                1.0,                    # vdop: Vertical dilution of precision
                0.0,                    # vn: Northing velocity (m/s) — stationary
                0.0,                    # ve: Easting velocity (m/s) — stationary
                0.0,                    # vd: Downward velocity (m/s) — stationary
                0.0,                    # speed_accuracy
                0.1,                    # horiz_accuracy (meters)
                0.1,                    # vert_accuracy (meters)
                int(num_sats),          # satellites_visible (displays in QGC)
                0                       # yaw (GPS heading; 0 if not available)
            )
        except Exception as e:
            self.get_logger().warn(f'GPS injection to ROV failed: {e}')

    def set_gps_global_origin(self, lat, lon, alt):
        """
        Sends SET_GPS_GLOBAL_ORIGIN MAVLink command to establish home position.
        This tells ArduSub: "Here is your starting position for AUTO mode."
        
        :param lat: Latitude in decimal degrees
        :param lon: Longitude in decimal degrees
        :param alt: Altitude/Height in meters (MSL)
        """
        try:
            if self.mavlink_master is None:
                return
            
            # Convert decimal degrees to integer format (deg * 1e7)
            lat_int = int(lat * 1e7)
            lon_int = int(lon * 1e7)
            
            # Send SET_GPS_GLOBAL_ORIGIN to set home position
            self.mavlink_master.mav.set_gps_global_origin_send(
                self.mavlink_master.target_system,      # target_system (autopilot)
                self.mavlink_master.target_component,   # target_component
                lat_int,                                # latitude (deg * 1e7)
                lon_int,                                # longitude (deg * 1e7)
                int(alt * 1000)                         # altitude (mm above MSL)
            )
            self.get_logger().info(f'SET_GPS_GLOBAL_ORIGIN sent: {lat:.6f}, {lon:.6f}, {alt:.1f}m')
        except Exception as e:
            self.get_logger().warn(f'SET_GPS_GLOBAL_ORIGIN failed: {e}')

    def send_snapshot(self):
        snapshot = {
            'timestamp': time.time(),
            'gps': self.latest['gps'],
            'battery': self.latest['battery'],
            'tension': self.latest['tension'],
            'encoder': self.latest['encoder'],
            'motors': self.latest['motors'],
            'rov_data': self.latest['rov_data'],
            'rov_position': self.latest['rov_position'],
        }

        # Publish on ROS topic
        msg = String()
        msg.data = json.dumps(snapshot)
        self.pub_state.publish(msg)

        # Send over UDP as JSON by default
        payload = msg.data.encode('utf-8')
        try:
            self.sock.sendto(payload, (self.udp_host, int(self.udp_port)))
        except Exception as e:
            self.get_logger().warn(f'UDP send failed: {e}')

        # Send MAVLink GLOBAL_POSITION_INT for both vehicles so QGC shows them on the map
        if self.send_mavlink:
            tboot = int(time.time() * 1000) & 0xFFFFFFFF  # ms, wraps at ~49 days

            # --- USV position (sysid=2, boat icon) ---
            if snapshot['gps']:
                try:
                    _fix, lat, lon, alt = snapshot['gps']
                    pkt = self.mav_usv.global_position_int_encode(
                        tboot,
                        int(lat * 1e7),   # lat  [1e-7 deg]
                        int(lon * 1e7),   # lon  [1e-7 deg]
                        int(alt * 1000),  # alt  [mm] above MSL
                        0,                # relative_alt [mm]
                        0, 0, 0,          # vx, vy, vz [cm/s] — unknown
                        65535,            # hdg [cdeg] — 65535 = unknown
                    ).pack(self.mav_usv)
                    self.sock.sendto(pkt, (self.udp_host, int(self.udp_port)))
                except Exception as e:
                    self.get_logger().warn(f'MAVLink USV position send failed: {e}')

            # --- Calculated ROV position (sysid=3, submarine icon) ---
            if snapshot['rov_position']:
                try:
                    rov_lat, rov_lon, rov_depth = snapshot['rov_position']
                    # alt above MSL: use USV altitude minus depth as a rough estimate
                    usv_alt_m = snapshot['gps'][3] if snapshot['gps'] else 0.0
                    rov_alt_mm = int((usv_alt_m - rov_depth) * 1000)
                    pkt = self.mav_rov_calc.global_position_int_encode(
                        tboot,
                        int(rov_lat * 1e7),
                        int(rov_lon * 1e7),
                        rov_alt_mm,
                        int(-rov_depth * 1000),  # relative_alt negative = below surface
                        0, 0, 0,
                        65535,
                    ).pack(self.mav_rov_calc)
                    self.sock.sendto(pkt, (self.udp_host, int(self.udp_port)))
                except Exception as e:
                    self.get_logger().warn(f'MAVLink calc ROV position send failed: {e}')

    def _send_heartbeats(self):
        """Send MAVLink HEARTBEAT at 1 Hz for both virtual vehicles so QGC registers them."""
        # MAVLink type/autopilot constants (integer values, no import needed)
        MAV_TYPE_SURFACE_BOAT = 7
        MAV_TYPE_SUBMARINE    = 12
        MAV_AUTOPILOT_GENERIC = 0
        MAV_STATE_ACTIVE      = 4

        try:
            pkt_usv = self.mav_usv.heartbeat_encode(
                MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE
            ).pack(self.mav_usv)
            self.sock.sendto(pkt_usv, (self.udp_host, int(self.udp_port)))

            pkt_rov = self.mav_rov_calc.heartbeat_encode(
                MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE
            ).pack(self.mav_rov_calc)
            self.sock.sendto(pkt_rov, (self.udp_host, int(self.udp_port)))
        except Exception as e:
            self.get_logger().warn(f'Heartbeat send failed: {e}')

    def _request_home_position(self):
        """Ask ArduSub / QGC for the currently set home point. Response handled in send_snapshot loop."""
        if not self.send_mavlink:
            return
        try:
            self.sock  # not the MAVLink socket — we need a proper mav object
            # Home position is requested via the MAVLink connection stored in usv_remote_server.
            # Here we just listen for HOME_POSITION broadcasts that ArduSub sends periodically.
            # The active request is done once in start_ros() via request_home_position().
            pass
        except Exception:
            pass

    def publish_home_position(self, lat_deg: float, lon_deg: float):
        """Called externally (from usv_remote_server) after receiving HOME_POSITION from ArduSub."""
        self._home_pub.publish(Float32MultiArray(data=[float(lat_deg), float(lon_deg)]))
        self.get_logger().info(f'Home position published: {lat_deg:.6f}, {lon_deg:.6f}')

    def _video_forward_loop(self):
        """
        Listen for raw UDP video packets on self.video_listen_port and forward
        them unmodified to the configured RF host/port. This is a single-channel
        unprocessed video passthrough used to send ROV video to the RF antenna.
        """
        try:
            recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # increase receive buffer for large video UDP bursts
            try:
                recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            except Exception:
                pass
            recv_sock.bind(('0.0.0.0', self.video_listen_port))
            recv_sock.settimeout(1.0)
            send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._video_send_sock = send_sock
            while not self._video_stop.is_set():
                try:
                    data, addr = recv_sock.recvfrom(65536)
                    # Forward unmodified packet to RF host
                    send_sock.sendto(data, (self.rf_host, self.rf_port))
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().warn(f"Video forwarder loop error: {e}")
                    time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f"Video forwarder failed to start: {e}")
        finally:
            try:
                recv_sock.close()
            except Exception:
                pass
            if self._video_send_sock:
                try:
                    self._video_send_sock.close()
                except Exception:
                    pass
            self.get_logger().info("Video forwarder stopped.")

    def destroy_node(self):
        # Stop video forwarder if running
        try:
            self._video_stop.set()
            if self._video_thread and self._video_thread.is_alive():
                self._video_thread.join(timeout=1.0)
            if self._video_send_sock:
                try:
                    self._video_send_sock.close()
                except Exception:
                    pass
        except Exception:
            pass

        super().destroy_node()
        try:
            self.sock.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    # defaults: UDP JSON to 192.168.1.100:14550
    node = USVSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
