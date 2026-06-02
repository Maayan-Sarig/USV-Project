import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import smbus2
import time
import math
from collections import deque

class IMU(Node):
    def __init__(self):
        super().__init__('IMU')

        self.bus = smbus2.SMBus(1)
        self.address = 0x68
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake MPU6050

        self.location = self.create_publisher(Float32MultiArray, 'location', 10)
        self.create_subscription(Float32MultiArray, 'gps', self.gps_callback, 10)
        self.get_logger().info("Connected to GPS.")

        self.timer = self.create_timer(0.05, self.read_sensor)  # 20 Hz

        self.last_time = time.time()
        self.pitch = 0.0
        self.roll = 0.0
        self.alpha = 0.98

        self.last_ax = None
        self.last_read_time = time.time()

        self.window_size = 5
        self.ax_buffer = deque(maxlen=self.window_size)
        self.ay_buffer = deque(maxlen=self.window_size)
        self.az_buffer = deque(maxlen=self.window_size)

        # variables for GPS integration
        self.initial_lat = None
        self.initial_lon = None
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.new_gps_available = False

        self.meters_per_deg_lat = 111000.0
        self.meters_per_deg_lon = 111000.0

        # IMU Position Dead Reckoning
        self.imu_x = 0.0
        self.imu_y = 0.0
        self.vx = 0.0
        self.vy = 0.0

        self.estimated_x = 0.0
        self.estimated_y = 0.0

    def gps_callback(self, msg):
        # קריאת האינדקסים מתוך המערך ששודר ב-GPS.py
        fix = msg.data[0]
        lat = msg.data[1]
        lon = msg.data[2]
        alt = msg.data[3]

        # עדכון אך ורק אם יש קליטה אמיתית (Fix 3 או 4) והנתון אינו אפס
        if fix < 3 or lat == 0.0:
            return

        if self.initial_lat is None:
            self.initial_lat = lat
            self.initial_lon = lon
            self.gps_x = 0.0
            self.gps_y = 0.0
            self.imu_x = 0.0
            self.imu_y = 0.0
            self.vx = 0.0
            self.vy = 0.0
        else:
            delta_lat = lat - self.initial_lat
            delta_lon = lon - self.initial_lon
            
            self.gps_y = delta_lat * self.meters_per_deg_lat
            # התיקון הקריטי: קוסינוס של קו הרוחב (lat) במעלות רדיאניות
            self.gps_x = delta_lon * self.meters_per_deg_lon * math.cos(math.radians(lat))
            self.new_gps_available = True

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        value = (high << 8) | low
        if value > 32768:
            value = value - 65536
        return value

    def read_sensor(self):
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            if dt <= 0:
                return
            self.last_time = current_time

            # Read Raw Accel
            raw_ax = self.read_raw_data(0x3B)
            raw_ay = self.read_raw_data(0x3D)
            raw_az = self.read_raw_data(0x3F)

            # Read Raw Gyro
            raw_gx = self.read_raw_data(0x43)
            raw_gy = self.read_raw_data(0x45)
            raw_gz = self.read_raw_data(0x47)

            # Convert to physical units
            ax = raw_ax / 16384.0
            ay = raw_ay / 16384.0
            az = raw_az / 16384.0

            gx = raw_gx / 131.0
            gy = raw_gy / 131.0
            gz = raw_gz / 131.0

            self.monitor_sensor_freeze(ax, ay, az)

            # Moving average filter
            self.ax_buffer.append(ax)
            self.ay_buffer.append(ay)
            self.az_buffer.append(az)

            avg_ax = sum(self.ax_buffer) / len(self.ax_buffer)
            avg_ay = sum(self.ay_buffer) / len(self.ay_buffer)
            avg_az = sum(self.az_buffer) / len(self.az_buffer)

            # Dead Reckoning Integration
            self.vx += avg_ax * 9.81 * dt
            self.vy += avg_ay * 9.81 * dt

            self.imu_x += self.vx * dt
            self.imu_y += self.vy * dt

            # Complementary Filter / Sensor Fusion
            if self.new_gps_available:
                self.estimated_x = 0.7 * self.gps_x + 0.3 * self.imu_x
                self.estimated_y = 0.7 * self.gps_y + 0.3 * self.imu_y
                
                # Reset IMU drift to match fusion
                self.imu_x = self.estimated_x
                self.imu_y = self.estimated_y
                self.new_gps_available = False
            else:
                self.estimated_x = self.imu_x
                self.estimated_y = self.imu_y

            # Publish updated location to the system
            self.location.publish(Float32MultiArray(data=[self.estimated_x, self.estimated_y, avg_az]))

        except Exception as e:
            self.get_logger().error(f"I2C Error: {e}")
            self.try_reconnect()

    def try_reconnect(self):
        try:
            self.bus.close()
            time.sleep(0.5)
            self.bus = smbus2.SMBus(1)
            for _ in range(3):
                try:
                    self.bus.write_byte_data(self.address, 0x6B, 0)
                    self.get_logger().info("I2C reconnected successfully.")
                    return
                except:
                    time.sleep(0.5)
            self.get_logger().error("Failed to wake up MPU6050 after reconnect.")
        except Exception as e:
            self.get_logger().error(f"Total I2C reconnect failure: {e}")

    def monitor_sensor_freeze(self, ax, ay, az):
        if self.last_ax is None:
            self.last_ax = (ax, ay, az)
            self.last_read_time = time.time()
            return
        if (abs(ax - self.last_ax[0]) < 0.01 and
            abs(ay - self.last_ax[1]) < 0.01 and
            abs(az - self.last_ax[2]) < 0.01):
            if time.time() - self.last_read_time > 2.0:
                self.get_logger().error("IMU freeze detected! Trying to reset...")
                self.try_reconnect()
        else:
            self.last_ax = (ax, ay, az)
            self.last_read_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
