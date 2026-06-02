# tension_node.py
import RPi.GPIO as GPIO
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TensionNode(Node):
    def __init__(self, dt_pin=22, sck_pin=27, calibration_factor=15290.5, mag=100, max_th=30):
        super().__init__('tension_node')
        self.dt_pin = dt_pin
        self.sck_pin = sck_pin
        self.calibration_factor = calibration_factor
        self.mag = mag
        self.max_th = max_th

        self.readings = []
        self.danger_array = []
        self.diff = []
        self.tension = 0
        self.filtered_val = 0.0
        self.danger_time = time.time()
        # Important to know if its safe to start the whole system
        self.calibration_failed = False

        self.cal_error = 0 # Counts the number of not optimal clibration

        # Kalman Filter parameters
        self.kf_x = 0.0        # Estimated state
        self.kf_P = 1.0        # Estimated error covariance
        self.kf_Q = 0.01       # Process noise covariance
        self.kf_R = 0.05       # Measurement noise covariance (tune this to match noise level)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dt_pin, GPIO.IN)
        GPIO.setup(self.sck_pin, GPIO.OUT)

        self.publisher_ = self.create_publisher(Float32, 'tension', 10)

        self.calibrate()
        self.timer = self.create_timer(0.05, self.check_weight)

    def read_hx711(self):
        while GPIO.input(self.dt_pin) == 1:
            pass

        count = 0
        for _ in range(24):
            GPIO.output(self.sck_pin, 1)
            count = count << 1
            GPIO.output(self.sck_pin, 0)
            if GPIO.input(self.dt_pin):
                count += 1

        GPIO.output(self.sck_pin, 1)
        GPIO.output(self.sck_pin, 0)

        if count & 0x800000:
            count -= 0x1000000

        return count

    def scale_value(self, raw):
        return -raw / self.calibration_factor

    def calibrate(self):
        print("Calibrating...")
        while len(self.readings) < 100:
            raw = self.read_hx711()
            scaled = self.scale_value(raw)

            if len(self.readings) == 2:
                if abs(scaled - self.readings[-1]) < 1.5:
                    self.readings.append(scaled)
                else:
                    self.readings.clear()
            elif len(self.readings) > 2:
                if abs(scaled - self.readings[-1]) < 1.5:
                    self.readings.append(scaled)
            else:
                self.readings.append(scaled)

            time.sleep(1 / self.mag)

        self.filtered_val = sum(self.readings) / len(self.readings)
        if self.filtered_val > 30 or self.filtered_val < -10:
            print(f"Calibration conditions were not optimal (Value is: {self.filtered_val:.2f}), Calibration restarted")
            self.cal_error +1
            self.filtered_val = 0.0
            self.calibrate()
            if self.cal_error > 5:
                self.get_logger().error("FAILED TO CALIBRATE, TRY AGAIN AT BETTER CONDITIONS")
                self.calibration_failed = True
                raise KeyboardInterrupt  # Trigger shutdown from main
        self.readings = []
        print(f"Calibration complete. Tared value: {self.filtered_val:.2f}")
        # Publishes the tared value
        msg = Float32()
        msg.data = float(self.filtered_val)
        self.publisher_.publish(msg)

    def check_weight(self):
        raw = self.read_hx711()
        scaled = self.scale_value(raw) - self.filtered_val
        self.diff.append(scaled)

        if len(self.diff) >= 2:
            if 140 <= abs(scaled) <= 500:
                if not self.danger_array:
                    self.danger_time = time.time()
                self.danger_array.append(scaled)

                if len(self.danger_array) >= 4:
                    if time.time() - self.danger_time < 5:
                        self.tension = np.mean(self.danger_array)
                        self.get_logger().warn(f"DANGER! {self.danger_array}")
                        self.danger_array = []
                    else:
                        self.danger_time = time.time()
                        self.danger_array = []
            elif abs(self.diff[1] - self.diff[0]) <= self.max_th:
                self.readings.append(scaled)
                if len(self.readings) >= 5:
                    self.tension = np.mean(self.readings)
            self.diff = self.diff[1:]

        if self.tension != 0:
            msg = Float32()
            msg.data = float(self.tension)
            self.publisher_.publish(msg)
            self.readings = []
            self.tension = 0

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()