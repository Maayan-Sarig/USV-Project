from pymavlink import mavutil
import sys

# 1. Establish connection
# Ensure MAVProxy is forwarding to 127.0.0.1:14551
connection = mavutil.mavlink_connection('udp:127.0.0.1:14551')

print("Connecting to vehicle...")
connection.wait_heartbeat()
print("Connected! System ID: %u, Component ID: %u" % (connection.target_system, connection.target_component))

# 2. Get list of supported modes
supported_modes = list(connection.mode_mapping().keys())

def set_mode(mode_name):
    # Check if mode exists
    if mode_name not in connection.mode_mapping():
        print(f"Error: Mode '{mode_name}' is not supported.")
        print(f"Supported modes are: {', '.join(supported_modes)}")
        return False

    # Get mode ID
    mode_id = connection.mode_mapping()[mode_name]

    # Send MAVLink command
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"Successfully sent command: Switch to {mode_name}")
    return True

# 3. Main interactive loop
print("\n--- Manual Submarine Control ---")
print(f"Available modes: {', '.join(supported_modes)}")
print("Type 'exit' to quit.")

try:
    while True:
        # Get user input from terminal
        user_input = input("\nEnter desired mode: ").upper().strip()

        if user_input == 'EXIT':
            print("Exiting...")
            break
        
        if user_input:
            set_mode(user_input)

except KeyboardInterrupt:
    print("\nProgram terminated.")