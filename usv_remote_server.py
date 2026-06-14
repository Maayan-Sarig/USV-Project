#!/usr/bin/env python3
import argparse
import json
import socket
import threading
import time

from pymavlink import mavutil

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from hx711 import TensionNode
    from stepper import WinchMotorNode
    from t200 import ThrusterNode
    from imu import IMU
    from GPS import GPS
    from logger import Logger
    from follower import Follower
    from ENCODER import encoder as EncoderNode
    from cmd_vel_bridge import CmdVelBridge
    from joy_to_cmd_vel import JoyToCmdVel
    from state_aggregator import USVStateNode
    from usv_sensor_bridge import USVSensorBridge
    from rov_position import ROVPositionNode
    from cruise_control import CruiseControlNode
    from station_keeping import StationKeepingNode
    from rtl import RTLNode
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False

from blue_rov2_terminal_control import (
    connect,
    show_modes,
    set_mode,
    arm,
    rc_override,
    manual_control,
    set_depth,
    set_heading,
    set_location,
    set_velocity,
    gimbal_control,
    set_servo,
    send_command,
    listen_type,
    request_status,
    show_status_once,
    set_stream,
    request_one_message,
    upload_mission,
    start_mission,
    request_mission_list,
    set_relay,
    request_battery,
    request_home_position,
)


class VideoForwarder:
    def __init__(self, local_port, remote_host, remote_port):
        self.local_port = int(local_port)
        self.remote_host = remote_host
        self.remote_port = int(remote_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.local_port))
        self.running = threading.Event()
        self.thread = None

    def _run(self):
        self.running.set()
        self.sock.settimeout(1.0)
        while self.running.is_set():
            try:
                data, addr = self.sock.recvfrom(65536)
                if data:
                    self.sock.sendto(data, (self.remote_host, self.remote_port))
            except socket.timeout:
                continue
            except Exception:
                continue

    def start(self):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def stop(self):
        self.running.clear()
        if self.thread:
            self.thread.join(timeout=1.0)
        try:
            self.sock.close()
        except Exception:
            pass


class RemoteCommandServer:
    def __init__(self, mav_connection, state_node=None, host='0.0.0.0', port=15000):
        self.master = mav_connection
        self.state_node = state_node
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.running = False
        self.video_forwarder = None
        self.last_client_addr = None
        self._mode_pub = None   # set by USVService after ROS init

    def handle_command(self, command_text):
        parts = command_text.strip().split()
        if not parts:
            return 'Empty command'

        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd == 'help':
                return self.help_text()
            elif cmd == 'modes':
                show_modes(self.master)
                return 'Modes displayed on server console.'
            elif cmd == 'mode':
                # Check for USV operating mode first
                USV_MODES = ('STATION_KEEPING', 'MANUAL', 'AUTO', 'RTL')
                if args and args[0].upper() in USV_MODES:
                    new_mode = args[0].upper()
                    if self._mode_pub is not None:
                        from std_msgs.msg import String as RosString
                        self._mode_pub.publish(RosString(data=new_mode))
                    return f'USV mode set to {new_mode}'
                # Otherwise treat as ROV MAVLink mode
                if not args:
                    return f'Usage: mode <ROV_MODE> or mode <{"│".join(USV_MODES)}>'
                set_mode(self.master, args[0])
                return f'ROV mode command sent: {args[0]}'
            elif cmd == 'arm':
                arm(self.master, True)
                return 'Arm command sent.'
            elif cmd == 'disarm':
                arm(self.master, False)
                return 'Disarm command sent.'
            elif cmd == 'rc':
                if not args:
                    return 'Usage: rc <ch1> ... <ch8>'
                rc_override(self.master, args)
                return f'RC override sent: {args}'
            elif cmd == 'manual':
                if len(args) < 4:
                    return 'Usage: manual <x> <y> <z> <r>'
                manual_control(self.master, *args[:4])
                return f'MANUAL_CONTROL sent: {args[:4]}'
            elif cmd == 'depth':
                if not args:
                    return 'Usage: depth <m>'
                set_depth(self.master, args[0])
                return f'Depth target sent: {args[0]}'
            elif cmd == 'heading':
                if not args:
                    return 'Usage: heading <deg>'
                set_heading(self.master, args[0])
                return f'Heading target sent: {args[0]}'
            elif cmd == 'position':
                if len(args) < 3:
                    return 'Usage: position <north> <east> <down> [yaw]'
                set_location(self.master, *args)
                return f'Position target sent: {args}'
            elif cmd == 'velocity':
                if len(args) < 3:
                    return 'Usage: velocity <vx> <vy> <vz> [yaw_rate]'
                set_velocity(self.master, *args)
                return f'Velocity target sent: {args}'
            elif cmd == 'gimbal':
                if len(args) != 3:
                    return 'Usage: gimbal <pitch> <roll> <yaw>'
                gimbal_control(self.master, *args)
                return f'Gimbal command sent: {args}'
            elif cmd == 'servo':
                if len(args) != 2:
                    return 'Usage: servo <channel> <pwm>'
                set_servo(self.master, *args)
                return f'Servo command sent: {args}'
            elif cmd == 'light':
                if len(args) != 2:
                    return 'Usage: light <relay> <on|off>'
                relay = int(args[0])
                state = args[1].lower() in ('1', 'on', 'true', 'yes')
                set_relay(self.master, relay, state)
                return f'Light command sent relay={relay} state={state}'
            elif cmd == 'battery':
                request_battery(self.master)
                return 'Battery request sent.'
            elif cmd == 'mission':
                if not args:
                    return 'Usage: mission <clear|start|list>'
                if args[0] == 'clear':
                    self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
                    return 'Mission clear requested.'
                elif args[0] == 'start':
                    start_mission(self.master)
                    return 'Mission start requested.'
                elif args[0] == 'list':
                    items = request_mission_list(self.master)
                    return json.dumps(items)
                return 'Usage: mission <clear|start|list>'
            elif cmd == 'polygon':
                if len(args) < 5 or len(args[1:]) % 2 != 0:
                    return 'Usage: polygon <alt> <lat1> <lon1> <lat2> <lon2> ...'
                alt = float(args[0])
                coords = [float(x) for x in args[1:]]
                waypoints = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
                success = upload_mission(self.master, waypoints, alt=alt)
                return 'Polygon mission uploaded.' if success else 'Polygon upload failed.'
            elif cmd == 'command':
                if not args:
                    return 'Usage: command <id> [p1..p7]'
                send_command(self.master, args[0], args[1:])
                return f'Command {args[0]} sent.'
            elif cmd == 'stream':
                if len(args) < 2:
                    return 'Usage: stream <id> <rate>'
                set_stream(self.master, args[0], args[1])
                return f'Stream request sent: {args}'
            elif cmd == 'listen':
                if not args:
                    return 'Usage: listen <TYPE>'
                threading.Thread(target=listen_type, args=(self.master, args[0].upper()), daemon=True).start()
                return f'Listening for {args[0].upper()} on server console.'
            elif cmd == 'status':
                show_status_once(self.master)
                return 'Status requested.'
            elif cmd == 'telemetry':
                if self.state_node is None:
                    return 'Telemetry unavailable: ROS state node not running.'
                state = self.state_node.get_state()
                return json.dumps(state)
            elif cmd == 'video':
                if not args:
                    return 'Usage: video <start|stop|status> [local_port remote_host remote_port]'
                action = args[0].lower()
                if action == 'start':
                    if len(args) >= 4:
                        local_port = int(args[1])
                        remote_host = args[2]
                        remote_port = int(args[3])
                    elif self.last_client_addr is not None:
                        local_port = int(args[1]) if len(args) >= 2 else 5600
                        remote_host = self.last_client_addr[0]
                        remote_port = int(args[2]) if len(args) >= 3 else 5600
                    else:
                        return 'Remote host required when no client address is known.'
                    self.start_video_forwarder(local_port, remote_host, remote_port)
                    return f'Video forwarding started {local_port} -> {remote_host}:{remote_port}'
                elif action == 'stop':
                    self.stop_video_forwarder()
                    return 'Video forwarding stopped.'
                elif action == 'status':
                    if self.video_forwarder and self.video_forwarder.thread and self.video_forwarder.thread.is_alive():
                        return f'Video forwarding active on port {self.video_forwarder.local_port} -> {self.video_forwarder.remote_host}:{self.video_forwarder.remote_port}'
                    return 'Video forwarding inactive.'
                else:
                    return 'Usage: video <start|stop|status> [local_port remote_host remote_port]'
            else:
                return f'Unknown command: {cmd}. {self.help_text()}'
        except Exception as exc:
            return f'Error executing command {cmd}: {exc}'

    def help_text(self):
        return (
            'help, modes, mode <MODE>, arm, disarm, rc <ch1>...<ch8>, manual <x> <y> <z> <r>, '
            'depth <m>, heading <deg>, position <N> <E> <D> [yaw], velocity <vx> <vy> <vz> [yaw_rate], '
            'gimbal <pitch> <roll> <yaw>, servo <chan> <pwm>, light <relay> <on|off>, battery, '
            'mission <clear|start|list>, polygon <alt> <lat lon ...>, command <id> [p1..p7], stream <id> <rate>, listen <TYPE>, telemetry, video <start|stop|status>, status'
        )

    def run(self):
        self.running = True
        print(f'Remote command server listening on udp://{self.host}:{self.port}')
        while self.running:
            data, addr = self.sock.recvfrom(4096)
            if not data:
                continue
            self.last_client_addr = addr
            text = data.decode('utf-8', errors='ignore').strip()
            if not text:
                continue
            print(f'Received from {addr}: {text}')
            response = self.handle_command(text)
            self.sock.sendto(response.encode('utf-8'), addr)

    def start_video_forwarder(self, local_port, remote_host, remote_port):
        if self.video_forwarder:
            self.video_forwarder.stop()
        self.video_forwarder = VideoForwarder(local_port, remote_host, remote_port)
        self.video_forwarder.start()

    def stop_video_forwarder(self):
        if self.video_forwarder:
            self.video_forwarder.stop()
            self.video_forwarder = None

    def stop(self):
        self.running = False
        self.sock.close()


class USVService:
    def __init__(self, mavlink_conn_str, command_port, ros_enable=False):
        self.mavlink_conn_str = mavlink_conn_str
        self.command_port = command_port
        self.ros_enable = ros_enable
        self.ros_nodes = []
        self.executor = None
        self.ros_thread = None
        self.server = None
        self.state_node = None
        self.mav = None  # Created before ROS so ROVPositionNode can receive it

    def start_ros(self):
        if not ROS_AVAILABLE:
            raise RuntimeError('ROS2 is not available in this environment.')
        rclpy.init()
        self.state_node = USVStateNode()
        sensor_bridge = USVSensorBridge(udp_host='127.0.0.1', udp_port=14551, send_mavlink=True)
        rtl_node = RTLNode(mav_connection=self.mav)
        self.ros_nodes = [
            TensionNode(),
            WinchMotorNode(),
            ThrusterNode(),
            IMU(),
            GPS(),
            Logger(),
            Follower(),
            CruiseControlNode(),
            EncoderNode(),
            CmdVelBridge(),
            JoyToCmdVel(),
            self.state_node,
            ROVPositionNode(mav_connection=self.mav),
            sensor_bridge,
            StationKeepingNode(),
            rtl_node,
        ]
        self.executor = MultiThreadedExecutor()
        for node in self.ros_nodes:
            self.executor.add_node(node)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        # Request home position from QGC/ArduSub and distribute to ROS nodes
        try:
            home_lat, home_lon = request_home_position(self.mav)
            if home_lat is not None:
                sensor_bridge.publish_home_position(home_lat, home_lon)
                print(f'Home position from QGC: {home_lat:.6f}, {home_lon:.6f}')
            else:
                print('Warning: no HOME_POSITION received from ArduSub — nodes will auto-set from first GPS fix.')
        except Exception as e:
            print(f'Warning: could not request home position: {e}')

        print('ROS2 nodes started in USV service.')

    def stop_ros(self):
        if self.executor:
            self.executor.shutdown()
        for node in self.ros_nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()

    def run(self):
        self.mav = connect(self.mavlink_conn_str)
        if self.ros_enable:
            self.start_ros()
        self.server = RemoteCommandServer(self.mav, state_node=self.state_node, port=self.command_port)

        # Wire up ROS publisher for usv_mode so mode command works from UDP
        if self.ros_enable and ROS_AVAILABLE:
            import rclpy as _rclpy
            from std_msgs.msg import String as _RosString
            from rclpy.node import Node as _Node

            class _ModePub(_Node):
                def __init__(self):
                    super().__init__('_usv_mode_pub')
                    self.pub = self.create_publisher(_RosString, 'usv_mode', 10)

            _mp = _ModePub()
            self.executor.add_node(_mp)
            self.server._mode_pub = _mp.pub
        try:
            self.server.run()
        except KeyboardInterrupt:
            print('Shutting down USV service...')
        finally:
            self.server.stop()
            if self.ros_enable:
                self.stop_ros()


def main():
    parser = argparse.ArgumentParser(description='USV remote server for MAVLink and ROS2 control')
    parser.add_argument('--mavlink', default='udp:127.0.0.1:14551', help='MAVLink connection string for BlueROV2')
    parser.add_argument('--port', type=int, default=15000, help='UDP port to listen for remote commands')
    parser.add_argument('--ros', action='store_true', help='Start local ROS2 nodes on the Pi')
    args = parser.parse_args()

    service = USVService(args.mavlink, args.port, ros_enable=args.ros)
    service.run()


if __name__ == '__main__':
    main()
