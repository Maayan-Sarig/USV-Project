# GPS Signal Not Showing in QGroundControl - Troubleshooting Guide

## Root Cause Identified
Your GPS data was being published to ROS2 but **not forwarded to QGC via MAVLink**. The system lacked:
1. Active GPS-to-MAVLink forwarding
2. Correct coordinate format conversion
3. Sensor bridge in the ROS node pipeline

## Solutions Applied

### Step 1: Enable MAVLink GPS Forwarding ✓
- Modified `usv_sensor_bridge.py` to enable `send_mavlink=True` by default
- Added `USVSensorBridge` node to the ROS2 executor in `usv_remote_server.py`
- Set correct destination: `127.0.0.1:14551` (the vehicle's MAVLink connection)

### Step 2: Fixed GPS Coordinate Format ✓
- UBX GPS module provides lat/lon in degrees × 1e-7 format
- Updated GPS.py to convert: `lat_deg = lat / 1e7` before publishing
- This ensures MAVLink receives valid degree coordinates

### Step 3: MAVLink GPS Message Flow ✓
```
GPS.py (reads UBX) 
  → ROS2 'gps' topic [fix, lat, lon, alt]
    → USVSensorBridge (subscribes)
      → gps_raw_int_encode() (MAVLink format)
        → Vehicle via 14551
          → QGC receives and displays
```

## How to Test

### 1. Start the USV service with ROS enabled:
```bash
cd ~/USV
python3 usv_remote_server.py --mavlink udp:127.0.0.1:14551 --ros
```

### 2. Verify GPS reading in ROS2:
```bash
ros2 topic echo /gps
```
You should see: `data: [fix_type, latitude, longitude, altitude]`

### 3. Monitor MAVLink GPS messages:
```bash
# In another terminal, listen for GPS_RAW_INT messages
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:127.0.0.1:14551')
while True:
    msg = m.recv_match(type='GPS_RAW_INT', blocking=True)
    print(msg)
"
```

### 4. In QGroundControl:
- Connect to `udp:0.0.0.0:14550` (this will connect to your Pi)
- GPS signal should appear in the attitude display
- Check Vehicle > Analyze > Messages for GPS_RAW_INT

## If GPS Still Not Showing in QGC

### Check 1: GPS Serial Connection
```bash
# List USB devices
ls -la /dev/ttyACM* /dev/ttyUSB*

# Monitor raw serial data
python3 -c "
import serial
from pyubx2 import UBXReader
s = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ubr = UBXReader(s)
for i in range(10):
    raw, msg = ubr.read()
    print(f'Fix: {msg.fixType if hasattr(msg, \"fixType\") else \"N/A\"} | Lat: {msg.lat if hasattr(msg, \"lat\") else \"N/A\"}')
"
```

### Check 2: ROS2 GPS Topic Publishing
```bash
# Monitor the GPS topic
ros2 topic echo /gps --once --json
```
Output should show: `data: [1, 123456789, 987654321, 50.0]` (fix, lat*1e7, lon*1e7, alt)

### Check 3: MAVLink Connection
```bash
netstat -tulpn | grep 14551
# Should show LISTEN on port 14551
```

### Check 4: Python Import Issues
```bash
cd ~/USV
python3 -c "from usv_sensor_bridge import USVSensorBridge; print('✓ Import OK')"
python3 -c "from GPS import GPS; print('✓ Import OK')"
```

## Configuration Options

### QGC Connection Methods

**Method A: Direct USB (if vehicle is on same network)**
```
QGC → Connect → Add new connection
Type: UDP
Address: <RPi-IP>
Port: 14550
```

**Method B: Radio/Network**
```
QGC → Settings → Comm Links → Manage
Add UDP connection on port 14550
```

### Adjust GPS Update Rate
Edit `GPS.py` line ~18:
```python
# Change from 200ms (5Hz) to 100ms (10Hz)
set_msg = UBXMessage('CFG', 'CFG-RATE', 1, measRate=100, navRate=1, timeRef=0)
```

## Common Issues & Fixes

| Symptom | Cause | Fix |
|---------|-------|-----|
| GPS shows 0,0,0 | Coordinate conversion error | Restart ROS with new GPS.py |
| GPS in QGC says "No Fix" | GPS module still acquiring | Wait 60 seconds for first fix |
| MAVLink message types missing | send_mavlink=False | Check line in usv_sensor_bridge.py |
| "Connection refused" on 14551 | USV service not running | Run `python3 usv_remote_server.py --ros` |
| GPS jumps around | Coordinate format doubling | Verify GPS.py lat/lon division by 1e7 |

## Architecture Diagram
```
┌─────────────────────┐
│   GPS Module (UBX)  │
│   /dev/ttyACM0      │
│   9600 baud         │
└──────────┬──────────┘
           │
           │ Raw UBX bytes
           ▼
    ┌─────────────────────┐
    │   GPS.py Node       │
    │ Reads: lat/lon*1e7  │
    │ Publishes: /gps     │
    └──────────┬──────────┘
               │
               │ ROS2 topic
               │ [fix, lat_deg, lon_deg, alt]
               ▼
    ┌─────────────────────────────┐
    │ USVSensorBridge Node        │
    │ Subscribes to /gps          │
    │ send_mavlink=True           │
    └──────────┬──────────────────┘
               │
               │ MAVLink GPS_RAW_INT
               │ (UDP 127.0.0.1:14551)
               ▼
    ┌─────────────────────┐
    │  Vehicle MAVLink    │
    │  Connection         │
    └──────────┬──────────┘
               │
               │ MAVLink relay
               │ (UDP *.0.0.0:14550)
               ▼
    ┌─────────────────────┐
    │  QGroundControl     │
    │  GPS Display        │
    └─────────────────────┘
```

## Next Steps
1. Run `python3 usv_remote_server.py --ros` to start
2. Verify GPS topic with `ros2 topic echo /gps`
3. Connect QGC and confirm GPS signal appears
4. If issues persist, run Check 1-4 above

---
**Reference**: UBX u-blox GPS documentation, MAVLink GPS_RAW_INT specification
