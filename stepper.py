import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio
import time
import math

class WinchMotorNode(Node):
    def __init__(self):
        super().__init__('winch_motor_node')
        self.pi = pigpio.pi()

        self.step_pin = 5  # Any GPIO
        self.dir_pin = 6
        self.freq = 100_000  # 100kHz step pulse
        self.pulse_width = int(1_000_000 / (self.freq*2))  # microseconds
        self.tension_threshold = 35  # N or kgf depending on your HX711 scaling
        self.max_tension = 135
        self.previous_tension = 0.0
        self.current_freq = 0.0
        self.steps = 0.0
        #test
        self.motor_running = False
        self.time_started = 0.0
        self.rpm = 0.0

        # values for after taring the hx711
        self.tared = False
        self.tared_threshold = 0
        self.max_tared = 0

        self.pi.set_mode(self.step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)

        self.create_subscription(Float32, 'tension', self.tension_callback, 10)

        # publishes spin to track
        self.spin = self.create_publisher(Float32, 'stepper', 10)

    def tension_callback(self, msg):
        current_tension = msg.data

        if self.tared == False:
            self.max_tared = self.max_tension - msg.data
            self.tared_threshold = self.tension_threshold - msg.data
            self.tarde = True

        # takes into account the noise of the sensor
        if current_tension < 20:

            self.pi.write(self.dir_pin, 0)  # Forward
            # makes sure there is a gradual speedup and slowing down for the engine
            if current_tension >= 1:
                #self.rpm = self.start_motor((20-current_tension)/20, 1)
                self.rpm = self.start_motor(current_tension, 1)
                #self.rpm = self.start_motor(0.05, 1)
            # for the max tension and above, the max speed will apply
            else:
                self.rpm = self.start_motor(1, 1)

        elif current_tension > self.tension_threshold:
            
            self.pi.write(self.dir_pin, 1)  # Backward
            # makes sure there is a gradual speedup and slowing down for the engine
            if current_tension*2 <= self.max_tension:
                #self.rpm = self.start_motor((current_tension*2)/self.max_tension, -1)
                self.rpm = self.start_motor(current_tension, -1)
                #self.rpm = self.start_motor(0.05, -1)
            # for the max tension and above, the max speed will apply
            else:
                self.rpm = self.start_motor(1, -1)

        else:
            self.stop_motor()
            self.motor_running = False

        length = Float32()
        if (self.pi.read(self.dir_pin) == 0):
            length.data = self.rpm * -1
        else:
            length.data = self.rpm
        self.spin.publish(length)
            

    def start_motor(self, per, step):
        # Clamp per between reasonable limits
        per = max(0.1, min(per, 1.0))  # per must be between 10% and 100%
        #adjusted_freq = self.freq * per
        
        if step == -1:
            k=0.1
            adjusted_freq = self.freq / (1 + math.exp(-k * (per - 35)))
        else:
            k=0.1
            adjusted_freq = self.freq / (1 + math.exp(-k * (20 - per)))

        adjusted_freq = max(100, adjusted_freq)  # Avoid super low frequency
        self.current_freq = adjusted_freq

        micros_per_pulse = int(1_000_000 / adjusted_freq)
        new_width = int(micros_per_pulse / 2)  # 50% duty cycle

        # Safety check: Make sure timings make sense
        if new_width < 2:
            new_width = 2
        if micros_per_pulse - new_width < 2:
            micros_per_pulse = new_width * 2
        
        now = time.time()

        # If a previous wave was running, calculate how long it ran
        if self.motor_running:        
            time_elapsed = now - self.time_started
            pulses_sent = int(self.current_freq * time_elapsed)
            # calculates the RPM
            self.rpm = pulses_sent/32000.0 *60 / time_elapsed # From the stepper Data-Sheet
            self.steps += pulses_sent * step # Depends on the direction

            # Now update with the new wave/frequency
            self.time_started = now
            self.motor_running = True
            self.current_freq = adjusted_freq
        
        else:
            self.time_started = time.time()
            self.motor_running = True

        # Build a new simple square pulse wave
        pulses = [
            pigpio.pulse(1 << self.step_pin, 0, new_width),
            pigpio.pulse(0, 1 << self.step_pin, micros_per_pulse - new_width)
        ]

        self.pi.wave_add_generic(pulses)
        wave_id = self.pi.wave_create()
        self.pi.wave_send_repeat(wave_id)
        return self.rpm

    def stop_motor(self):
        self.pi.wave_tx_stop()
        self.pi.wave_clear()
        
        if self.motor_running:
            time_elapsed = time.time() - self.time_started
            estimated_steps = int(self.current_freq * time_elapsed)
            self.steps += estimated_steps if self.pi.read(self.dir_pin) == 1 else -estimated_steps
            self.motor_running = False

        self.rpm = 0.0
        self.pi.wave_tx_stop()
        self.pi.wave_clear()
        return self.rpm


    def destroy_node(self):
        self.pi.stop()
        super().destroy_node()