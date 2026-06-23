import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from pyubx2 import UBXReader, UBXMessage
import socket

class GPS(Node):
    def __init__(self):
        super().__init__('GPS')
        
        # Publisher for GPS data
        self.publisher_ = self.create_publisher(Float32MultiArray, 'gps', 10)

        # Set up serial connection to GPS
        port = '/dev/ttyACM0'  # USB virtual serial port
        self.stream = serial.Serial(port, baudrate=9600, timeout=1)
        
        self.ubr = UBXReader(self.stream)

        # Set to 5 Hz (200 ms)
        set_msg = UBXMessage('CFG', 'CFG-RATE', 1, measRate=200, navRate=1, timeRef=0)
        self.stream.write(set_msg.serialize())

        # Network connection setup to laptop (clean antenna network)
        self.laptop_ip = '192.168.2.1'
        self.udp_port = 14451
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Timer to periodically poll the GPS
        self.timer = self.create_timer(0.2, self.read_gps_data)

    def read_gps_data(self):
        try:
            raw_data, msg = self.ubr.read()
            if msg and hasattr(msg, 'identity') and msg.identity == 'NAV-PVT':
                lat = msg.lat
                lon = msg.lon
                alt = msg.height / 1000.0
                fix = msg.fixType
                
                # Publish to ROS2
                self.publisher_.publish(Float32MultiArray(data=[fix, lat, lon, alt]))

                # --- Convert UBX to NMEA for QGC ---
                lat_deg = abs(lat)
                lon_deg = abs(lon)
                
                lat_nmea = f"{int(lat_deg):02d}{(lat_deg % 1 * 60):07.4f}"
                lat_dir = 'N' if lat >= 0 else 'S'
                
                lon_nmea = f"{int(lon_deg):03d}{(lon_deg % 1 * 60):07.4f}"
                lon_dir = 'E' if lon >= 0 else 'W'
                
                nmea_sentence = f"GNGGA,120000.00,{lat_nmea},{lat_dir},{lon_nmea},{lon_dir},{fix},08,0.9,{alt:.1f},M,0.0,M,,"
                
                checksum = 0
                for char in nmea_sentence:
                    checksum ^= ord(char)
                
                full_nmea = f"${nmea_sentence}*{checksum:02X}\r\n"
                
                # Forward to port 14401 on laptop via antennas
                self.sock.sendto(full_nmea.encode('ascii'), (self.laptop_ip, self.udp_port))

        except Exception as e:
            self.get_logger().warn(f"Error reading GPS: {e}")

    def destroy_node(self):
        super().destroy_node()
        if hasattr(self, 'stream'):
            self.stream.close()
        if hasattr(self, 'sock'):
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    gps_node = GPS()
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
