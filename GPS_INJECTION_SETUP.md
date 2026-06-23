# MAVLink GPS_INPUT Injection Setup

## Overview
The `usv_sensor_bridge.py` now forwards USV GPS coordinates to the ROV's autopilot via MAVLink `GPS_INPUT` messages. This feeds the EKF and enables QGroundControl to display the injected GPS fix and satellite count.

## Implementation Details

### GPS Injection Flow
1. **GPS Node** (`GPS.py`) publishes `Float64MultiArray` on ROS topic `/gps` with format:
   ```
   [fix_type, latitude_decimal, longitude_decimal, altitude_meters]
   ```

2. **Sensor Bridge** (`usv_sensor_bridge.py`) receives GPS callback and:
   - Checks if `fix_type >= 2` (valid 2D or 3D fix)
   - Converts latitude/longitude to MAVLink integer format (degrees × 1e7)
   - Sends `GPS_INPUT` message with:
     - Timestamp in microseconds (epoch)
     - GPS ID = 0 (primary virtual GPS instance)
     - HDOP/VDOP = 1.0 (excellent horizontal/vertical precision)
     - Velocities = 0 (USV assumed stationary relative to ROV)
     - Satellites visible = 15 (hardcoded for QGC display)
     - Yaw = 0 (no GPS heading available)

3. **ArduSub Autopilot** (after configuration):
   - Receives `GPS_INPUT` as virtual MAVLink GPS device
   - Feeds coordinates to EKF3 navigation filter
   - Generates `GPS_RAW_INT` telemetry for QGC
   - Locks position for navigation modes (AUTO, GUIDED, etc.)

4. **QGroundControl** displays:
   - Green satellite icon (15 satellites) in top bar
   - USV position on map
   - EKF lock status

## Critical ArduSub Parameters (Must Be Set in QGC)

### 1. GPS_TYPE = 14 (MAV)
**What it does:** Tells the autopilot: "Your primary GPS is coming as MAVLink `GPS_INPUT` packets, not a hardware serial module."

**Where to set:**
- QGC → Vehicle Setup → Parameters → GPS (tab)
- Search for `GPS_TYPE` and set to `14`

### 2. EK3_SRC1_POSXY = 3 (GPS)
**What it does:** Forces the Extended Kalman Filter 3 to use GPS for horizontal position, not compass drift or dead reckoning.

**Where to set:**
- QGC → Vehicle Setup → Parameters → EKF3 (tab)
- Search for `EK3_SRC1_POSXY` and set to `3`

### 3. GPS_DELAY_MS = 0
**What it does:** Eliminates time-sync delay between the RPi4 system clock and the autopilot's internal clock loop.

**Where to set:**
- QGC → Vehicle Setup → Parameters → GPS (tab)
- Search for `GPS_DELAY_MS` and set to `0`

---

## Configuration Procedure

1. **Connect ROV to QGroundControl** via MAVLink (14551 or tether)

2. **Set parameters in QGC:**
   ```
   GPS_TYPE       = 14
   EK3_SRC1_POSXY = 3
   GPS_DELAY_MS   = 0
   ```

3. **Reboot the ROV** (autopilot must restart for GPS_TYPE driver allocation to take effect)

4. **Start the USV/ROV ROS2 system:**
   ```bash
   ros2 launch usv_remote_server.py  # or your launch command
   ```

5. **Verify in QGC:**
   - Check the top bar for satellite count (should show 15)
   - Look for green GPS lock icon
   - USV position should appear on the map with coordinates from the bridge

---

## Monitoring & Debugging

### Check GPS Injection is Working:
```bash
# In a separate terminal, monitor the sensor bridge logs
ros2 run usv_sensor_bridge usv_sensor_bridge > /tmp/bridge.log 2>&1 &

# Look for messages like:
# [usv_sensor_bridge] GPS callback received fix_type=3 lat=31.265... lon=34.803...
```

### Verify MAVLink Traffic:
```bash
# Use a MAVLink sniffer (if available) to confirm GPS_INPUT packets are being sent
mavproxy.py --master udp:127.0.0.1:14551 --out udp:192.168.1.100:14550
# Look for GPS_INPUT message frames in the output
```

### Check EKF Status:
- QGC → Analyze → Messages (or Console)
- Look for `GPS_RAW_INT` messages being generated after `GPS_INPUT` injection starts
- Monitor `AHRS` or `EKF` status messages for convergence

---

## Troubleshooting

### QGC Shows No Satellites / No GPS Lock:
1. Verify `GPS_TYPE = 14` is set (not 1 or 2)
2. Ensure ROV was **rebooted** after changing `GPS_TYPE`
3. Check that `GPS_DELAY_MS = 0`
4. Confirm ROS2 system is running and GPS callback is firing (check logs)

### GPS_INPUT Messages Not Being Sent:
1. Verify `self.mavlink_master` is not None in the bridge (check logs)
2. Confirm GPS node is publishing valid data (fix_type ≥ 2)
3. Check MAVLink connection is established (should see heartbeats in QGC)

### EKF Not Locking / Position Drifting:
1. Verify `EK3_SRC1_POSXY = 3` is set
2. Check HDOP/VDOP (should be ≤ 2.0 for good convergence)
3. Ensure system time on RPi4 is reasonably accurate (NTP recommended)

---

## Future Enhancements

1. **Dynamic satellite count:** Extract actual count from u-blox GPS parser instead of hardcoding 15
2. **Velocity injection:** Use t200 thruster data to populate `vn`, `ve`, `vd` fields for better EKF convergence
3. **Multi-antenna GPS heading:** If dual-antenna setup is available, populate `yaw` field for course-over-ground
4. **Accuracy metrics:** Dynamically adjust `hdop`/`vdop` based on u-blox report

---

## Files Modified

- **usv_sensor_bridge.py**: Added `inject_gps_to_rov()` method, updated GPS callback, accepts `mav_connection` parameter
- **usv_remote_server.py**: Passes `self.mav` to sensor bridge constructor
- **GPS.py**: No changes (publishes Float64MultiArray with [fix, lat, lon, alt])

---

## Parameter Documentation Reference
- ArduSub Parameter List: https://ardupilot.org/copter/docs/parameters.html#gps-type
- MAVLink GPS_INPUT Message: https://mavlink.io/en/messages/common.html#GPS_INPUT
- EKF3 Configuration: https://ardupilot.org/copter/docs/ekf3-parameters.html
