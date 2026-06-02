import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import pigpio
import time
import math

# === Configuration ===
PWM_PIN_RIGHT = 26  # BCM GPIO pin (not physical number)
PWM_PIN_LEFT = 12   # BCM GPIO pin (not physical number)
FREQ = 50           # ESCs expect 50Hz
NEUTRAL = 1500      # in microseconds
FORWARD = 1600      # in microseconds
REVERSE = 1400      # in microseconds

class ThrusterNode(Node):
    def __init__(self):
        super().__init__('thruster_node')
        self.boost =  0.0

        self.right = pigpio.pi()
        self.left = pigpio.pi()
        if not (self.right.connected or self.left.connected):
            print("Could not connect to pigpiod")
            exit()

        self.right.set_mode(PWM_PIN_RIGHT, pigpio.OUTPUT)

        # === Arm the ESC at neutral ===
        print("Arming ESC (neutral)...")
        self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, NEUTRAL)
        self.left.set_servo_pulsewidth(PWM_PIN_LEFT, NEUTRAL)

        self.subscription1 = self.create_subscription(Float32, 'encoder_angle', self.thrust_callback, 10)
        self.subscription2 = self.create_subscription(Float32, 'thruster_cmd', self.thrust_callback, 10)
        self.subscription3 = self.create_subscription(Float32, 'follow', self.booster, 10)
        self.get_logger().info("Thruster node ready.")

        # publishes spin to track
        self.spin = self.create_publisher(Float32MultiArray, 't200_speed', 10)

    def booster(self, msg):
        self.boost = msg.data

    def thrust_callback(self, msg):
        forward = 0.0
        reverse = 0.0
        if msg.data < -10:
            if -msg.data <= 20:
                forward = NEUTRAL + 50 * ((-msg.data)/20.0)
                reverse = NEUTRAL - 50 * ((-msg.data)/20.0)
            else:
                forward = NEUTRAL + 50
                reverse = NEUTRAL - 50
            forward += self.boost
            reverse += self.boost
            self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, forward)
            self.left.set_servo_pulsewidth(PWM_PIN_LEFT, reverse)
            self.spin.publish(Float32MultiArray(data=[forward, reverse]))

        elif msg.data > 10:
            if msg.data <= 20:
                forward = NEUTRAL + 50*((msg.data)/20.0)
                reverse = NEUTRAL - 50*((msg.data)/20.0)
            else:
                forward = NEUTRAL + 50
                reverse = NEUTRAL - 50
            forward += self.boost
            reverse += self.boost
            self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, reverse)
            self.left.set_servo_pulsewidth(PWM_PIN_LEFT, forward)
            self.spin.publish(Float32MultiArray(data=[reverse, forward]))

        else:
            neutral = NEUTRAL + self.boost
            self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, neutral)
            self.left.set_servo_pulsewidth(PWM_PIN_LEFT, neutral)
            self.spin.publish(Float32MultiArray(data=[neutral, neutral]))

    def destroy_node(self):
        self.get_logger().info("Stopping T200s...")
        self.left.set_servo_pulsewidth(PWM_PIN_LEFT, 0)
        self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, 0)
        self.right.stop()
        self.left.stop()
        super().destroy_node()