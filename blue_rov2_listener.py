from pymavlink import mavutil
import time

# Connect to local port created by MAVProxy
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

while True:
    try:
        # Waiting for ATTITUDE message type
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            print(f"Roll: {round(msg.roll, 2)}, Pitch: {round(msg.pitch, 2)}, Yaw: {round(msg.yaw, 2)}")
        time.sleep(0.5)
    except KeyboardInterrupt:
        break
