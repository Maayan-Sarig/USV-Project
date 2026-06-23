import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float32
import tkinter as tk
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import subprocess

class Display(Node):
    def __init__(self):
        super().__init__('display_data')
        self.spins = 0.0
        self.dir = 0.0
        self.length = 0.0
        self.last_stepper_time = time.time()
        # Create the window
        self.window = tk.Tk()
        self.window.title("Sensor Data Display")
        self.window.geometry("900x350")

        # Create tension label
        self.tension_label = tk.Label(self.window, text="Tension (kg): Waiting...", font=("Arial", 16))
        self.tension_label.pack(pady=20)

        # Create angle label
        self.angle_label = tk.Label(self.window, text="Angle (deg): Waiting...", font=("Arial", 16))
        self.angle_label.pack(pady=20)

        # Create acceleration label
        self.accel_label = tk.Label(self.window, text="acceleration (m^2/sec): Waiting...", font=("Arial", 16))
        self.accel_label.pack(pady=20)

        # Create gps label
        self.gps_label = tk.Label(self.window, text="GPS: Waiting...", font=("Arial", 16))
        self.gps_label.pack(pady=20)

        # Create length label
        self.dep = tk.Label(self.window, text="depth calculated: ", font=("Arial", 16))
        self.dep.pack(pady=20)

        # ROS subscriptions
        self.subscription1 = self.create_subscription(Float32, 'tension', self.listener_callback1, 10)
        self.get_logger().info("Subscribed to 'tension'")

        self.subscription4 = self.create_subscription(Float64MultiArray, 'gps', self.update_gps, 10)
        self.get_logger().info("Subscribed to 'GPS'")

        self.subscription2 = self.create_subscription(Float32, 'encoder_angle', self.listener_callback2, 10)
        self.get_logger().info("Subscribed to 'encoder_angle'")

        self.subscription3 = self.create_subscription(Float32MultiArray, 'location', self.update_location, 10)
        self.get_logger().info("Subscribed to 'imu'")

        self.subscription4 = self.create_subscription(Float32, 'stepper', self.depth, 10)
        self.get_logger().info("Subscribed to 'Stepper'")

        # Periodic update for depth calculation
        self.create_timer(0.2, self.update_depth)

        # Start Video Forwarding (Transparent Bridge)
        # We use subprocess to run the socat command in the background
        self.start_video_bridge()

    def listener_callback1(self, msg):
        tension = msg.data
        self.tension_label.config(text=f"Tension (kg): {tension:.2f}")

    def listener_callback2(self, msg):
        angle = msg.data
        self.angle_label.config(text=f"Angle (deg): {angle:.2f}")

    def update_location(self, msg):
        x, y, z = msg.data
        self.accel_label.config(text=f"Location(m): x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def update_gps(self, msg):
        fix, lat, lon, alt = msg.data
        self.gps_label.config(text=f"Coordinates(m): x={lat:.7f}, y={lon:.7f}, z={alt:.2f}")
    
    def depth(self, msg):
        self.dir = msg.data
        self.last_stepper_time = time.time()

    def update_depth(self):
        # Stop spinning if no recent stepper message
        if time.time() - self.last_stepper_time > 1.0 or self.spins < 0.0:
            self.dir = 0.0
            self.spins = 0.0

        # Update spin and length
        self.spins += (self.dir / 60.0) * 0.2

        if abs(self.spins) > 23.125:
            n = int(self.spins / 23.125)
        else:
            n = 0

        partial_spins = self.spins % 23.125
        self.length = 2.0 * partial_spins * 3.14 * (0.095 + 0.0076 / 2.0 + (5 - n) * 0.0076)

        if n > 0:
            for i in range(n):
                self.length += 2.0 * 23.125 * 3.14 * (0.095 + 0.0076 / 2.0 + (5 - i) * 0.0076)
        dep_c = self.length/3 + self.length/3 * math.sin(45)

        self.dep.config(text=f"Depth Calculated (m): {dep_c:.2f}")

    def run(self):
        # Run ROS and Tkinter together
        try:
            while rclpy.ok():
                self.window.update()
                rclpy.spin_once(self)
        except tk.TclError:
            self.get_logger().info("GUI closed.")
        finally:
            self.destroy_node()

    def start_video_bridge(self):
        operator_ip = "192.168.137.1" # The IP of the PC with QGroundControl
        video_port = "5600"
        
        command = f"socat UDP4-LISTEN:{video_port},fork UDP4:{operator_ip}:{video_port}"
        
        try:
            # Start socat as a background process
            subprocess.Popen(command.split(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"Video bridge started: Port {video_port} -> {operator_ip}")
        except Exception as e:
            self.get_logger().error(f"Failed to start video bridge: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Display()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()