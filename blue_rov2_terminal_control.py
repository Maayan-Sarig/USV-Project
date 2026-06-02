#!/usr/bin/env python3
"""BlueROV2 terminal MAVLink controller.

This script connects to the BlueROV2 simulator over UDP and provides a
terminal interface for BlueROV2 / ArduSub commands, including mode
switching, arming, RC override, depth/yaw targets, gimbal control,
and generic MAV_CMD sending.
"""

import argparse
import sys
import time

from pymavlink import mavutil

DEFAULT_CONNECTION = 'udp:127.0.0.1:14551'


def connect(connection_str=DEFAULT_CONNECTION, timeout=30):
    master = mavutil.mavlink_connection(connection_str)
    print(f"Connecting to vehicle on {connection_str}...")
    master.wait_heartbeat(timeout=timeout)
    print(
        "Connected! Heartbeat received from system %u component %u"
        % (master.target_system, master.target_component)
    )
    return master


def show_modes(master):
    modes = sorted(master.mode_mapping().keys())
    print("Supported modes:")
    print("  " + ", ".join(modes))


def set_mode(master, mode_name):
    mode_name = mode_name.upper()
    if mode_name not in master.mode_mapping():
        print(f"Error: unsupported mode '{mode_name}'")
        show_modes(master)
        return False

    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"Sent mode change to {mode_name}")
    return True


def arm(master, arm_state=True):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if arm_state else 0.0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("Sent arm command" if arm_state else "Sent disarm command")


def rc_override(master, channels):
    values = []
    for i in range(8):
        if i < len(channels):
            values.append(int(channels[i]))
        else:
            values.append(65535)

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *values,
    )
    print(f"Sent RC override: {values}")


def manual_control(master, x, y, z, r, buttons=0):
    x = int(max(-1000, min(1000, float(x))))
    y = int(max(-1000, min(1000, float(y))))
    z = int(max(-1000, min(1000, float(z))))
    r = int(max(-1000, min(1000, float(r))))
    master.mav.manual_control_send(
        master.target_system,
        x,
        y,
        z,
        r,
        int(buttons),
    )
    print(f"Sent MANUAL_CONTROL x={x} y={y} z={z} r={r} buttons={buttons}")


def set_depth(master, depth_m):
    depth = float(depth_m)
    type_mask = 2043  # ignore x/y position, all velocities/accels, yaw/yaw_rate
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0,
        0,
        depth,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print(f"Sent depth target: {depth} m (down positive)")


def set_heading(master, yaw_deg):
    yaw = float(yaw_deg)
    type_mask = 1535  # ignore x/y/z pos, velocities, accels, yaw_rate
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        yaw,
        0,
    )
    print(f"Sent heading target: {yaw} degrees")


def set_location(master, north, east, down, yaw=None):
    x = float(north)
    y = float(east)
    z = float(down)
    if yaw is None:
        type_mask = 2040  # position only, ignore yaw/yaw_rate
    else:
        type_mask = 1528  # position and yaw
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        0,
        0,
        0,
        0,
        0,
        0,
        float(yaw) if yaw is not None else 0,
        0,
    )
    print(f"Sent position target N={x} E={y} D={z}" + (f" yaw={yaw}" if yaw is not None else ""))


def set_velocity(master, vx, vy, vz, yaw_rate=None):
    x = 0.0
    y = 0.0
    z = 0.0
    yaw = 0.0
    type_mask = 1023  # ignore position x/y/z and accel/force
    if yaw_rate is None:
        type_mask |= 512  # ignore yaw_rate
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        float(vx),
        float(vy),
        float(vz),
        0,
        0,
        0,
        yaw,
        float(yaw_rate) if yaw_rate is not None else 0,
    )
    print(f"Sent velocity target vx={vx} vy={vy} vz={vz}" + (f" yaw_rate={yaw_rate}" if yaw_rate is not None else ""))


def gimbal_control(master, pitch, roll, yaw):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,
        float(pitch),
        float(roll),
        float(yaw),
        0,
        0,
        0,
        0,
    )
    print(f"Sent gimbal control pitch={pitch} roll={roll} yaw={yaw}")


def set_servo(master, channel, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        int(channel),
        int(pwm),
        0,
        0,
        0,
        0,
        0,
    )
    print(f"Sent servo command channel={channel} pwm={pwm}")


def send_command(master, cmd_id, params):
    values = [float(p) for p in params]
    values += [0.0] * (7 - len(values))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        int(cmd_id),
        0,
        *values[:7],
    )
    print(f"Sent command {cmd_id} params={values[:7]}")


def set_relay(master, relay, state):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
        0,
        int(relay),
        1.0 if state else 0.0,
        0,
        0,
        0,
        0,
        0,
    )
    print(f"Sent relay command relay={relay} state={state}")


def upload_mission(master, waypoints, alt=2.0):
    count = len(waypoints)
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(0.5)
    master.mav.mission_count_send(master.target_system, master.target_component, count)
    print(f"Uploading mission with {count} waypoints")

    while True:
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=15)
        if msg is None:
            print("Mission upload timed out.")
            return False

        if msg.get_type() == 'MISSION_ACK':
            print("Mission upload complete", msg)
            return True

        seq = msg.seq
        if seq < 0 or seq >= count:
            print(f"Received mission request for invalid seq {seq}")
            continue

        lat, lon = waypoints[seq]
        if msg.get_type() == 'MISSION_REQUEST_INT':
            print(f"Sending MISSION_ITEM_INT seq={seq} lat={lat} lon={lon} alt={alt}")
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0,
                0,
                0,
                0,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt),
            )
        else:
            print(f"Sending MISSION_ITEM seq={seq} lat={lat} lon={lon} alt={alt}")
            master.mav.mission_item_send(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0,
                0,
                0,
                0,
                float(lat),
                float(lon),
                float(alt),
            )


def start_mission(master, first=0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        float(first),
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print(f"Sent mission start command first_item={first}")


def request_mission_list(master, timeout=10):
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    print('Requested mission list...')
    deadline = time.time() + timeout
    msg = None
    while time.time() < deadline:
        msg = master.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT', 'MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'MISSION_COUNT':
            count = msg.count
            print(f'Received mission count: {count}')
            if count == 0:
                return []
            items = []
            for seq in range(count):
                master.mav.mission_request_int_send(
                    master.target_system,
                    master.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                )
                item = None
                item_deadline = time.time() + 5
                while time.time() < item_deadline:
                    item = master.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM', 'MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=1)
                    if item is None:
                        continue
                    if item.get_type() in ('MISSION_ITEM_INT', 'MISSION_ITEM') and item.seq == seq:
                        lat = item.x if item.get_type() == 'MISSION_ITEM' else item.x / 1e7
                        lon = item.y if item.get_type() == 'MISSION_ITEM' else item.y / 1e7
                        alt = item.z
                        cmd = item.command
                        items.append((seq, cmd, lat, lon, alt))
                        print(f'Seq {seq}: cmd={cmd} lat={lat} lon={lon} alt={alt}')
                        break
                    # Some autopilots still request the next item type before sending it
                    if item.get_type() == 'MISSION_REQUEST_INT':
                        continue
                    if item.get_type() == 'MISSION_REQUEST':
                        continue
                else:
                    print(f'No response for mission item {seq}')
                    break
            return items
    print('Mission list request timed out.')
    return []


def listen_type(master, msg_type='ATTITUDE'):
    print(f"Listening for {msg_type} messages. Ctrl-C to stop.")
    try:
        while True:
            msg = master.recv_match(type=msg_type, blocking=True, timeout=5)
            if msg:
                print(msg)
    except KeyboardInterrupt:
        print("Stopped listening.")


def request_status(master):
    print("Requesting vehicle status messages...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        66,  # MAV_CMD_REQUEST_DATA_STREAM
        0,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        1,
        1,
        1,
        0,
        0,
        0,
    )
    print("Request sent. Use 'listen ATTITUDE' or 'listen SYS_STATUS' to read messages.")


def request_battery(master):
    print("Requesting battery status message...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    request_one_message(master, 'BATTERY_STATUS')


def print_status_message(msg_type, msg):
    print(f"--- {msg_type} ---")
    if hasattr(msg, 'to_dict'):
        fields = msg.to_dict()
    else:
        fields = getattr(msg, '__dict__', {})

    if msg_type == 'MOUNT_ORIENTATION':
        for name in ['time_boot_ms', 'roll', 'pitch', 'yaw', 'yaw_absolute']:
            if name in fields and name != 'type':
                print(f"{name}: {fields[name]}")
        print()
        return

    for name, value in fields.items():
        if name == 'type':
            continue
        print(f"{name}: {value}")
    print()


def show_status_once(master):
    print("Requesting one-time submarine status...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        66,  # MAV_CMD_REQUEST_DATA_STREAM
        0,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        1,
        1,
        1,
        0,
        0,
        0,
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_MOUNT_ORIENTATION,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    messages = {
        'HEARTBEAT': None,
        'SYS_STATUS': None,
        'ATTITUDE': None,
        'LOCAL_POSITION_NED': None,
        'VFR_HUD': None,
        'BATTERY_STATUS': None,
        'MOUNT_ORIENTATION': None,
    }
    deadline = time.time() + 5
    while time.time() < deadline and any(v is None for v in messages.values()):
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type in messages and messages[msg_type] is None:
            messages[msg_type] = msg

    if all(v is None for v in messages.values()):
        print("No status messages received.")
    else:
        for msg_type in [
            'HEARTBEAT',
            'SYS_STATUS',
            'BATTERY_STATUS',
            'MOUNT_ORIENTATION',
            'ATTITUDE',
            'VFR_HUD',
            'LOCAL_POSITION_NED',
        ]:
            msg = messages.get(msg_type)
            if msg is not None:
                print_status_message(msg_type, msg)

        missing = [k for k, v in messages.items() if v is None]
        if missing:
            print(f"Missing: {', '.join(missing)}")


def set_stream(master, stream_id, rate, start_stop=1):
    master.mav.command_long_send(
                                 
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_DATA_STREAM,
        0,
        int(stream_id),
        float(rate),
        int(start_stop),
        0,
        0,
        0,
        0,
    )
    print(f"Requested stream {stream_id} at {rate} Hz (start_stop={start_stop})")


def request_one_message(master, msg_type='LOCAL_POSITION_NED', timeout=2):
    print(f"Waiting for one {msg_type} message...")
    msg = master.recv_match(type=msg_type, blocking=True, timeout=timeout)
    if msg:
        print(msg)
    else:
        print(f"No {msg_type} received in {timeout}s.")


def print_help():
    print("\nAvailable commands:")
    print("  help                  Show this help text")
    print("  modes                 List supported flight modes")
    print("  mode <MODE>           Set vehicle mode")
    print("  arm                   Arm the vehicle")
    print("  disarm                Disarm the vehicle")
    print("  rc <ch1> ... <ch8>    Send RC override values (65535=no change)")
    print("  manual <x> <y> <z> <r>   Send MANUAL_CONTROL (-1000..1000)")
    print("  depth <m>             Set depth target (down positive)")
    print("  heading <deg>         Set yaw/heading target")
    print("  position <N> <E> <D> [yaw]   Set local NED position target")
    print("  velocity <vx> <vy> <vz> [yaw_rate]  Set local velocity target")
    print("  gimbal <pitch> <roll> <yaw>   Control camera mount")
    print("  servo <chan> <pwm>    Set servo output")
    print("  light <relay> <on|off> Set relay/light state")
    print("  battery               Request battery status")
    print("  mission clear         Erase current mission")
    print("  mission start         Start the active mission")
    print("  mission list          List current mission items")
    print("  polygon <alt> <lat1> <lon1> <lat2> <lon2> ...   Upload polygon mission")
    print("  command <id> [p1..p7] Send arbitrary MAV_CMD_COMMAND_LONG")
    print("  stream <id> <rate>    Request MAVLink stream data")
    print("  listen <TYPE>         Listen for MAVLink messages of TYPE")
    print("  status                Show one-time submarine status")
    print("  exit                  Quit")


def main():
    parser = argparse.ArgumentParser(description='BlueROV2 terminal MAVLink controller')
    parser.add_argument(
        '--connection',
        default=DEFAULT_CONNECTION,
        help='MAVLink connection string (default: %(default)s)',
    )
    args = parser.parse_args()

    try:
        master = connect(args.connection)
    except Exception as exc:
        print(f"Connection failed: {exc}")
        sys.exit(1)

    print_help()

    try:
        while True:
            line = input('\n> ').strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ('exit', 'quit'):
                break
            if cmd == 'help':
                print_help()
            elif cmd == 'modes':
                show_modes(master)
            elif cmd == 'mode':
                if not args:
                    print("Usage: mode <MODE>")
                    continue
                set_mode(master, args[0])
            elif cmd == 'arm':
                arm(master, True)
            elif cmd == 'disarm':
                arm(master, False)
            elif cmd == 'rc':
                if not args:
                    print("Usage: rc <ch1> ... <ch8>")
                    continue
                rc_override(master, args)
            elif cmd == 'manual':
                if len(args) < 4:
                    print("Usage: manual <x> <y> <z> <r>")
                    continue
                manual_control(master, *args[:4])
            elif cmd == 'depth':
                if not args:
                    print("Usage: depth <m>")
                    continue
                set_depth(master, args[0])
                request_one_message(master, 'LOCAL_POSITION_NED')
            elif cmd == 'heading':
                if not args:
                    print("Usage: heading <deg>")
                    continue
                set_heading(master, args[0])
            elif cmd == 'position':
                if len(args) < 3:
                    print("Usage: position <north> <east> <down> [yaw]")
                    continue
                set_location(master, *args)
            elif cmd == 'velocity':
                if len(args) < 3:
                    print("Usage: velocity <vx> <vy> <vz> [yaw_rate]")
                    continue
                set_velocity(master, *args)
            elif cmd == 'gimbal':
                if len(args) != 3:
                    print("Usage: gimbal <pitch> <roll> <yaw>")
                    continue
                gimbal_control(master, *args)
            elif cmd == 'servo':
                if len(args) != 2:
                    print("Usage: servo <channel> <pwm>")
                    continue
                set_servo(master, *args)
            elif cmd == 'light':
                if len(args) != 2:
                    print("Usage: light <relay> <on|off>")
                    continue
                relay = int(args[0])
                state = args[1].lower() in ('1', 'on', 'true', 'yes')
                set_relay(master, relay, state)
            elif cmd == 'battery':
                request_battery(master)
            elif cmd == 'mission':
                if not args:
                    print("Usage: mission <clear|start|list>")
                    continue
                if args[0] == 'clear':
                    master.mav.mission_clear_all_send(master.target_system, master.target_component)
                    print("Requested mission clear")
                elif args[0] == 'start':
                    start_mission(master)
                elif args[0] == 'list':
                    request_mission_list(master)
                else:
                    print("Usage: mission <clear|start|list>")
            elif cmd == 'polygon':
                if len(args) < 5 or len(args[1:]) % 2 != 0:
                    print("Usage: polygon <alt> <lat1> <lon1> <lat2> <lon2> ...")
                    continue
                alt = float(args[0])
                coords = [float(x) for x in args[1:]]
                waypoints = []
                for i in range(0, len(coords), 2):
                    waypoints.append((coords[i], coords[i+1]))
                if upload_mission(master, waypoints, alt=alt):
                    print("Polygon mission uploaded successfully.")
                else:
                    print("Polygon upload failed.")
            elif cmd == 'command':
                if not args:
                    print("Usage: command <id> [p1..p7]")
                    continue
                send_command(master, args[0], args[1:])
            elif cmd == 'stream':
                if len(args) < 2:
                    print("Usage: stream <id> <rate>")
                    continue
                set_stream(master, args[0], args[1])
            elif cmd == 'listen':
                if not args:
                    print("Usage: listen <TYPE>")
                    continue
                listen_type(master, args[0].upper())
            elif cmd == 'status':
                show_status_once(master)
            else:
                print(f"Unknown command: {cmd}")
                print_help()
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == '__main__':
    main()
