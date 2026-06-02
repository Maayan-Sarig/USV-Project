import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# === GPIO Pins (BOARD mode) ===
A_PIN = 24  # Channel A
B_PIN = 23  # Channel B
TICKS_PER_REV = 8192 

position = 0  # Global tick counter

def update_position(channel):
    global position
    a_val = GPIO.input(A_PIN)
    b_val = GPIO.input(B_PIN)

    if channel == A_PIN:
        if a_val == b_val:
            position += 1
        else:
            position -= 1
    else:  # channel == B_PIN
        if a_val != b_val:
            position += 1
        else:
            position -= 1

class encoder(Node):
    def __init__(self):
        super().__init__('encoder')

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=update_position)
        GPIO.add_event_detect(B_PIN, GPIO.BOTH, callback=update_position)

        # Create publisher
        self.publisher_ = self.create_publisher(Float32, 'encoder_angle', 10)

        # Create timer to publish every 0.2 seconds
        self.timer = self.create_timer(0.2, self.publish_angle)

    def publish_angle(self):
        degrees = position * (360.0 / TICKS_PER_REV) * (360/310)
        msg = Float32()
        msg.data = degrees
        self.publisher_.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()