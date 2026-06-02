# USV BlueROV2 Control Integration

This folder now contains a working integration between your laptop, Raspberry Pi, and BlueROV2 autopilot.

## Architecture

- Laptop runs `usv_remote_client.py`
- Raspberry Pi runs `usv_main.py` / `usv_remote_server.py`
- Pi connects to BlueROV2 via MAVLink
- Pi optionally starts ROS2 nodes for local sensor and actuator control
- Commands are sent from laptop to Pi over UDP
- Video forwarding can be enabled from Pi to laptop over UDP

## Key files

- `usv_main.py` - Pi startup wrapper
- `usv_remote_server.py` - Pi remote server for MAVLink+ROS control
- `usv_remote_client.py` - Laptop client for sending commands
- `blue_rov2_terminal_control.py` - MAVLink terminal controller and command library
- `cmd_vel_bridge.py` - ROS2 bridge from `cmd_vel` to thruster commands
- `joy_to_cmd_vel.py` - ROS2 joystick -> `cmd_vel` bridge
- `state_aggregator.py` - ROS2 state collection node for telemetry
- `run_fast.py` / `run_slow.py` - local ROS2 node startup scripts

## Run on the Raspberry Pi4

From `/home/lar/USV` on the Pi:

```bash
python3 usv_main.py --mavlink udp:127.0.0.1:14551 --port 15000 --ros
```

- `--mavlink` sets the BlueROV2 connection string
- `--port` sets the UDP command port for laptop control
- `--ros` starts local ROS2 nodes

## Run on the laptop

From `/home/lar/USV` on your laptop:

```bash
python3 usv_remote_client.py --host <pi-ip> --port 15000 --interactive
```

Then enter commands interactively.

## Supported remote commands

- `help`
- `modes`
- `mode <MODE>`
- `arm`
- `disarm`
- `rc <ch1> ... <ch8>`
- `manual <x> <y> <z> <r>`
- `depth <m>`
- `heading <deg>`
- `position <north> <east> <down> [yaw]`
- `velocity <vx> <vy> <vz> [yaw_rate]`
- `gimbal <pitch> <roll> <yaw>`
- `servo <chan> <pwm>`
- `light <relay> <on|off>`
- `battery`
- `mission clear`
- `mission start`
- `mission list`
- `polygon <alt> <lat1> <lon1> <lat2> <lon2> ...`
- `command <id> [p1..p7]`
- `stream <id> <rate>`
- `listen <TYPE>`
- `telemetry`
- `video start <port> <host> <remote_port>`
- `video stop`
- `video status`
- `status`

## ROS2 control options

With `--ros` on the Pi, the following ROS2 components are started:

- GPS node
- IMU node
- encoder node
- tension/HX711 node
- thruster node
- stepper node
- follower node
- logger node
- ROS2 `cmd_vel` bridge
- joystick-to-`cmd_vel` bridge
- state aggregator for telemetry

## What to do next

1. Confirm the Pi has ROS2 installed and can run the ROS nodes.
2. Confirm the BlueROV2 MAVLink connection string on the Pi.
3. Plug in or configure your joystick so the ROS `joy` topic is available.
4. Connect the laptop to the Pi network via antenna.
5. Run the Pi server and then the laptop client.

## Notes

- The system now supports both manual MAVLink control and automatic mission upload.
- Video forwarding is UDP-based and forwards traffic from the Pi to the laptop.
- `telemetry` returns the latest ROS2 state snapshot if ROS is running.
