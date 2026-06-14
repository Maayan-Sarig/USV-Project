import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
import pigpio
import time
import math

# === Configuration ===
PWM_PIN_RIGHT = 26  # BCM GPIO pin (not physical number)
PWM_PIN_LEFT = 12   # BCM GPIO pin (not physical number)
FREQ = 50           # ESCs expect 50Hz
NEUTRAL = 1500      # µs

# Proportional steering constants (derived from truster_test.py patterns)
DEAD_ZONE_DEG    = 15    # degrees — no steering correction inside this band
MAX_ANGLE_DEG    = 75    # degrees — full hard-turn correction at this angle
OUTER_FWD_OFFSET = 100   # µs above NEUTRAL for the outer motor during a turn (→ 1600)
MAX_DIFF_OFFSET  = 200   # µs total differential at MAX_ANGLE_DEG (outer-inner range)

class ThrusterNode(Node):
    def __init__(self):
        super().__init__('thruster_node')
        self.boost        = 0.0   # depth-follow boost from follower.py  → 'follow'
        self.cruise_boost = 0.0   # forward cruise boost from cruise_control.py → 'cruise_boost'
        self._mode        = 'STATION_KEEPING'  # default — steered by sk_angle
        self._sk_boost    = 0.0   # forward boost from station_keeping/rtl → 'sk_boost'

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

        self.subscription1 = self.create_subscription(Float32, 'encoder_angle', self.steer_callback,   10)
        self.subscription2 = self.create_subscription(Float32, 'thruster_cmd',  self.steer_callback,   10)
        self.subscription3 = self.create_subscription(Float32, 'follow',        self.follow_boost_cb,  10)
        self.subscription4 = self.create_subscription(Float32, 'cruise_boost',  self.cruise_boost_cb,  10)
        self.subscription5 = self.create_subscription(String,  'usv_mode',      self._on_mode,         10)
        self.subscription6 = self.create_subscription(Float32, 'sk_angle',      self._on_sk_angle,     10)
        self.subscription7 = self.create_subscription(Float32, 'sk_boost',      self._on_sk_boost,     10)
        self.get_logger().info("Thruster node ready.")

        # publishes PWM values so other nodes can observe speed
        self.spin = self.create_publisher(Float32MultiArray, 't200_speed', 10)

    def follow_boost_cb(self, msg: Float32):
        self.boost = float(msg.data)

    def cruise_boost_cb(self, msg: Float32):
        self.cruise_boost = float(msg.data)

    def _on_mode(self, msg: String):
        self._mode = msg.data
        self.get_logger().info(f"USV mode → {self._mode}")

    def _on_sk_angle(self, msg: Float32):
        if self._mode in ('STATION_KEEPING', 'RTL'):
            self._apply_steering(float(msg.data), self._sk_boost)

    def _on_sk_boost(self, msg: Float32):
        self._sk_boost = float(msg.data)

    def steer_callback(self, msg: Float32):
        """
        Proportional 3-zone steering from encoder_angle / thruster_cmd topics.
        Bypassed when mode is STATION_KEEPING or RTL (sk_angle used instead).

        Effective PWM differential at key angles (boost=0):
          angle ≈ 34° (norm=0.25) → outer=1600, inner=1550  → SOFT TURN
          angle ≈ 53° (norm=0.5)  → outer=1600, inner=1500  → inner neutral
          angle = 75° (norm=1.0)  → outer=1600, inner=1400  → HARD TURN
        """
        if self._mode in ('STATION_KEEPING', 'RTL'):
            return  # steered by sk_angle topic instead
        self._apply_steering(float(msg.data), self.boost + self.cruise_boost)

    def _apply_steering(self, angle: float, total_boost: float):
        """Shared proportional steering — used by both steer_callback and _on_sk_angle."""
        if abs(angle) < DEAD_ZONE_DEG:
            pwm = int(NEUTRAL + total_boost)
            right_us = pwm
            left_us  = pwm
        else:
            norm = min(1.0, (abs(angle) - DEAD_ZONE_DEG) / (MAX_ANGLE_DEG - DEAD_ZONE_DEG))
            outer_us = int(NEUTRAL + OUTER_FWD_OFFSET + total_boost)
            inner_us = int(outer_us - MAX_DIFF_OFFSET * norm)

            if angle > 0:   # cable to the RIGHT → turn right → right motor is inner
                right_us = inner_us
                left_us  = outer_us
            else:           # cable to the LEFT → turn left → left motor is inner
                right_us = outer_us
                left_us  = inner_us

        self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, right_us)
        self.left.set_servo_pulsewidth(PWM_PIN_LEFT,   left_us)
        self.spin.publish(Float32MultiArray(data=[float(right_us), float(left_us)]))

    def destroy_node(self):
        self.get_logger().info("Stopping T200s...")
        self.left.set_servo_pulsewidth(PWM_PIN_LEFT, 0)
        self.right.set_servo_pulsewidth(PWM_PIN_RIGHT, 0)
        self.right.stop()
        self.left.stop()
        super().destroy_node()