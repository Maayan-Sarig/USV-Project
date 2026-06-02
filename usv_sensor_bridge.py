import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
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


class USVSensorBridge(Node):
    def __init__(self, udp_host='192.168.1.100', udp_port=14550, send_mavlink=True):
        super().__init__('usv_sensor_bridge')

        self.udp_host = udp_host
        self.udp_port = udp_port
        self.send_mavlink = send_mavlink and MAVLINK_AVAILABLE

        self.latest = {
            'gps': None,        # [fix, lat, lon, alt]
            'battery': None,    # {'voltage':..., 'percent':...}
            'tension': None,    # float
            'encoder': None,    # degrees float
            'motors': None,     # list or tuple
            'rov_data': None,   # passthrough raw data
        }

        # Subscribers (match what's used in the repo)
        self.create_subscription(Float32MultiArray, 'gps', self.cb_gps, 10)
        self.create_subscription(Float32, 'tension', self.cb_tension, 10)
        self.create_subscription(Float32, 'encoder_angle', self.cb_encoder, 10)
        self.create_subscription(Float32MultiArray, 't200_speed', self.cb_motors, 10)
        # battery topic may not exist; listen on 'battery' if present
        self.create_subscription(Float32MultiArray, 'battery', self.cb_battery, 10)

        # Publish aggregated state as JSON on ROS topic
        self.pub_state = self.create_publisher(String, 'usv/state', 10)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # MAVLink setup
        if self.send_mavlink:
            self.get_logger().info('pymavlink available: sending MAVLink messages')
            self.mav = mavutil.mavlink.MAVLink(None)
            # default sysid/component
            self.mav.srcSystem = 1
            self.mav.srcComponent = 1
        elif not MAVLINK_AVAILABLE and send_mavlink:
            self.get_logger().warn('Requested MAVLink send but pymavlink not available — falling back to JSON')

        # Timer to send snapshot
        self.timer = self.create_timer(0.2, self.send_snapshot)

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
        except Exception:
            pass

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

    def send_snapshot(self):
        snapshot = {
            'timestamp': time.time(),
            'gps': self.latest['gps'],
            'battery': self.latest['battery'],
            'tension': self.latest['tension'],
            'encoder': self.latest['encoder'],
            'motors': self.latest['motors'],
            'rov_data': self.latest['rov_data'],
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

        # Optionally send minimal MAVLink messages (GPS and battery)
        if self.send_mavlink:
            try:
                if snapshot['gps']:
                    fix, lat, lon, alt = snapshot['gps']
                    # convert to MAVLink GPS_RAW_INT format
                    time_usec = int(time.time() * 1e6)
                    lat_i = int(lat * 1e7)
                    lon_i = int(lon * 1e7)
                    alt_mm = int(alt * 1000)
                    msg_gps = self.mav.gps_raw_int_encode(time_usec, int(fix or 0), lat_i, lon_i, alt_mm, 0, 0, 0, 0, 0)
                    pkt = msg_gps.pack(self.mav)
                    self.sock.sendto(pkt, (self.udp_host, int(self.udp_port)))

                if snapshot['battery']:
                    volt = int(snapshot['battery'].get('voltage', 0) * 1000)
                    # create a SYS_STATUS message with battery voltage in millivolts
                    msg_sys = self.mav.sys_status_encode(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                    pkt2 = msg_sys.pack(self.mav)
                    self.sock.sendto(pkt2, (self.udp_host, int(self.udp_port)))
            except Exception as e:
                self.get_logger().warn(f'MAVLink send failed: {e}')

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
