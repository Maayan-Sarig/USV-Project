import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from pyubx2 import UBXReader, UBXMessage

class GPS(Node):
    def __init__(self):
        super().__init__('GPS')
        
        # Publisher for GPS data
        self.publisher_ = self.create_publisher(Float32MultiArray, 'gps', 10)

        # Set up serial connection to GPS
        port = '/dev/ttyACM0'  # USB virtual serial port
        self.stream = serial.Serial(port, baudrate=9600, timeout=1)  # baudrate is ignored but required by pyserial API
        
        self.ubr = UBXReader(self.stream)

        # Set to 5 Hz (200 ms)
        set_msg = UBXMessage('CFG', 'CFG-RATE', 1, measRate=200, navRate=1, timeRef=0)
        self.stream.write(set_msg.serialize())

        # Timer to periodically poll the GPS
        self.timer = self.create_timer(0.2, self.read_gps_data)

    def read_gps_data(self):
        try:
            raw_data, msg = self.ubr.read()
            if msg and hasattr(msg, 'identity') and msg.identity == 'NAV-PVT':
                # Divide by 10000000.0 to convert raw integer to standard degrees
                lat = msg.lat / 10000000.0
                lon = msg.lon / 10000000.0
                alt = msg.height / 1000.0
                fix = msg.fixType
                
                # Publish data to ROS2 topic
                self.publisher_.publish(Float32MultiArray(data=[float(fix), float(lat), float(lon), float(alt)]))

        except Exception as e:
            self.get_logger().warn(f"Error reading GPS: {e}")

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
