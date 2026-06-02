#!/usr/bin/env python3
import pigpio
import time
import sys

PIN_RIGHT = 26
PIN_LEFT  = 12

NEUTRAL    = 1500
SLOW_FWD   = 1550
GENTLE_FWD = 1600
FAST_FWD   = 1700
GENTLE_REV = 1400
FAST_REV   = 1300

HOLD       = 2
HOLD_NEUTR = 2
ARM_HOLD   = 3

pi = pigpio.pi()
if not pi.connected:
    print("ERROR: pigpiod is not running. Run: sudo pigpiod")
    sys.exit(1)

def drive(right_us, left_us, hold_seconds, label):
    print(f"[{label}]  R={right_us}  L={left_us}  hold {hold_seconds} s")
    pi.set_servo_pulsewidth(PIN_RIGHT, right_us)
    pi.set_servo_pulsewidth(PIN_LEFT,  left_us)
    time.sleep(hold_seconds)

def neutral():
    drive(NEUTRAL, NEUTRAL, HOLD_NEUTR, "neutral")

try:
    drive(NEUTRAL, NEUTRAL, ARM_HOLD, "ARM")
    drive(GENTLE_FWD, GENTLE_FWD, HOLD, "GENTLE FORWARD"); neutral()
    drive(FAST_FWD,   FAST_FWD,   HOLD, "FAST FORWARD");   neutral()
    drive(GENTLE_REV, GENTLE_REV, HOLD, "GENTLE REVERSE"); neutral()
    drive(FAST_REV,   FAST_REV,   HOLD, "FAST REVERSE");   neutral()
    drive(GENTLE_REV, GENTLE_FWD, HOLD, "HARD TURN RIGHT"); neutral()
    drive(GENTLE_FWD, GENTLE_REV, HOLD, "HARD TURN LEFT");  neutral()
    drive(SLOW_FWD,   FAST_FWD,   HOLD, "SOFT TURN RIGHT"); neutral()
    drive(FAST_FWD,   SLOW_FWD,   HOLD, "SOFT TURN LEFT");  neutral()
finally:
    print("Stopping signals...")
    pi.set_servo_pulsewidth(PIN_RIGHT, 0)
    pi.set_servo_pulsewidth(PIN_LEFT,  0)
    pi.stop()
    print("Done.")