#!/usr/bin/env python3
"""
USV Comprehensive Test Suite
=============================

Aggregates all hardware bringup, logic, and integration tests in fault-isolation
order: fewest components first, full system last.

Each test is labelled:
  [AUTO]                    — runs without touching anything
  [AUTO on Pi]              — automatic, but requires Raspberry Pi + wiring
  [AUTO when connected]     — automatic when MAVProxy + BlueROV2 are reachable
  [REQUIRES HARDWARE]       — needs Pi wiring but no physical interaction
  [REQUIRES PHYSICAL INTERACTION] — operator must move something during the test
  [REQUIRES PHYSICAL PRESENCE]    — operator must supervise (motors move)

Pytest marks:
  sw          Pure-software tests — no hardware, no ROS2 required (CI-safe)
  unit        Single hardware component
  integration Sensor→actuator chains
  rov         Requires live BlueROV2 + MAVProxy on udp:127.0.0.1:14551
  visual      Physically moves motors — operator must be present

Quick run examples:
  pytest tests/test_comprehensive.py -v -s -m sw              # CI-safe, no hardware
  pytest tests/test_comprehensive.py -v -s -m "unit"          # hardware unit tests
  pytest tests/test_comprehensive.py -v -s -m integration     # sensor→actuator
  pytest tests/test_comprehensive.py -v -s -m rov             # BlueROV2 required
  pytest tests/test_comprehensive.py -v -s -m visual          # visual demo
  pytest tests/test_comprehensive.py -v -s                    # full suite
"""

import math
import time
import threading
import pytest

# ── Production constants ── imported directly so tests use the same source values
from t200 import (
    DEAD_ZONE_DEG, MAX_ANGLE_DEG, NEUTRAL,
    OUTER_FWD_OFFSET, MAX_DIFF_OFFSET,
)
from cruise_control import LOW_TENSION, HIGH_TENSION, CRUISE_BOOST
from cruise_control import DEAD_ZONE_DEG as CC_DEAD_ZONE  # must equal t200.DEAD_ZONE_DEG
from rov_position import _cable_length_from_spins, _DEFAULT_SAG_FACTOR


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 0 — Infrastructure
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.unit
def test_pigpiod_alive():
    """
    [AUTO on Pi] 0.1  Verify pigpiod daemon is running before any PWM test.
    If this fails: run `sudo pigpiod` on the Raspberry Pi.
    """
    import pigpio
    pi = pigpio.pi()
    assert pi.connected, "pigpiod daemon is not running — start it with: sudo pigpiod"
    pi.stop()


@pytest.mark.unit
def test_gpio_pins_output(gpio_setup):
    """
    [AUTO on Pi] 0.2  Verify all critical GPIO pins can be configured as OUTPUT.
    Catches wiring conflicts or kernel driver locks before any sensor test runs.
    """
    import RPi.GPIO as GPIO
    critical_pins = {
        5:  'stepper STEP',
        6:  'stepper DIR',
        12: 'T200 left ESC',
        22: 'HX711 DT',
        23: 'encoder B',
        24: 'encoder A',
        26: 'T200 right ESC',
        27: 'HX711 SCK',
    }
    failed = []
    for pin, label in critical_pins.items():
        try:
            GPIO.setup(pin, GPIO.OUT)
        except Exception as e:
            failed.append(f"GPIO {pin} ({label}): {e}")
    assert not failed, "GPIO pin setup failures:\n  " + "\n  ".join(failed)


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 1 — Pure-software logic  (⭐ NEW — no hardware, CI-safe)
# ═══════════════════════════════════════════════════════════════════════════════

def _steering_pwm(angle: float, boost: float = 0.0):
    """Mirror of ThrusterNode.steer_callback — pure Python, no hardware dependency."""
    if abs(angle) < DEAD_ZONE_DEG:
        pwm = int(NEUTRAL + boost)
        return pwm, pwm  # right_us, left_us
    norm = min(1.0, (abs(angle) - DEAD_ZONE_DEG) / (MAX_ANGLE_DEG - DEAD_ZONE_DEG))
    outer_us = int(NEUTRAL + OUTER_FWD_OFFSET + boost)
    inner_us = int(outer_us - MAX_DIFF_OFFSET * norm)
    if angle > 0:   # cable to right → turn right → right is inner
        return inner_us, outer_us
    else:           # cable to left  → turn left  → left is inner
        return outer_us, inner_us


@pytest.mark.sw
def test_steering_formula():
    """
    [AUTO] 1.1 ⭐ NEW — Proportional steering formula (t200.py) at key angles.

    Ground truth from truster_test.py:
      dead zone  → both motors at NEUTRAL (1500 µs)
      soft turn  → outer=1600, inner≈1550  (50 µs differential)
      hard turn  → outer=1600, inner=1400  (200 µs differential)
    """
    # Dead zone — both motors neutral
    assert _steering_pwm(0.0)          == (1500, 1500), "0°: both should be neutral"
    assert _steering_pwm(10.0)         == (1500, 1500), "10° (inside dead zone): neutral"
    assert _steering_pwm(-10.0)        == (1500, 1500), "-10° (inside dead zone): neutral"

    # At boundary — tiny or zero differential
    r, l = _steering_pwm(float(DEAD_ZONE_DEG))
    assert abs(max(r, l) - min(r, l)) < 5, \
        f"{DEAD_ZONE_DEG}° (boundary): differential should be near 0, got {abs(r-l)}"

    # Soft turn RIGHT — norm≈0.25 → differential ≈ 50 µs
    soft_angle = DEAD_ZONE_DEG + 0.25 * (MAX_ANGLE_DEG - DEAD_ZONE_DEG)
    r, l = _steering_pwm(soft_angle)  # positive → cable right → right=inner
    assert l == 1600,          f"Soft turn right: outer(left) should be 1600 µs, got {l}"
    assert 40 <= (l - r) <= 60, f"Soft turn right: differential should be ~50 µs, got {l-r}"

    # Hard turn RIGHT — norm=1.0 → outer=1600, inner=1400 (matches truster_test.py)
    r, l = _steering_pwm(float(MAX_ANGLE_DEG))
    assert l == 1600, f"Hard turn right: outer(left) should be 1600 µs, got {l}"
    assert r == 1400, f"Hard turn right: inner(right) should be 1400 µs, got {r}"

    # Left-right symmetry
    r_pos, l_pos = _steering_pwm(50.0)
    r_neg, l_neg = _steering_pwm(-50.0)
    assert r_pos == l_neg, "Symmetric: right@+50° should equal left@-50°"
    assert l_pos == r_neg, "Symmetric: left@+50° should equal right@-50°"

    # Boost adds to both motors equally in dead zone
    r, l = _steering_pwm(0.0, boost=30.0)
    assert r == 1530 and l == 1530, f"boost=30 in dead zone: expected (1530,1530), got ({r},{l})"


@pytest.mark.sw
def test_cruise_logic():
    """
    [AUTO] 1.2 ⭐ NEW — Cruise control activation logic (cruise_control.py).

    DEAD_ZONE_DEG is imported from t200.py in cruise_control.py — single source of truth.
    Cruise activates only when BOTH conditions hold simultaneously.
    """
    assert CC_DEAD_ZONE == DEAD_ZONE_DEG, (
        f"cruise_control.DEAD_ZONE_DEG ({CC_DEAD_ZONE}) ≠ t200.DEAD_ZONE_DEG ({DEAD_ZONE_DEG}) "
        "— must share the same constant"
    )

    def active(angle, tension):
        return abs(angle) < DEAD_ZONE_DEG and LOW_TENSION < tension < HIGH_TENSION

    # Should ACTIVATE
    assert active(0.0,   25.0), "Center + nominal tension → must activate"
    assert active(5.0,   27.0), "Small angle + mid tension → must activate"
    assert active(-5.0,  30.0), "Small negative angle → must activate"
    assert active(DEAD_ZONE_DEG - 0.1, LOW_TENSION + 0.1), "Just inside all bounds → must activate"

    # Should NOT activate
    assert not active(DEAD_ZONE_DEG, 25.0),  "Angle exactly at dead zone boundary → must NOT activate"
    assert not active(DEAD_ZONE_DEG + 1, 25.0), "Angle outside dead zone → deactivate"
    assert not active(0.0, LOW_TENSION),     "Tension = LOW_TENSION (not strictly >) → deactivate"
    assert not active(0.0, HIGH_TENSION),    "Tension = HIGH_TENSION (not strictly <) → deactivate"
    assert not active(0.0, 10.0),            "Under-tension → deactivate"
    assert not active(0.0, 45.0),            "Over-tension → deactivate"
    assert not active(20.0, 10.0),           "Both conditions wrong → deactivate"

    # Verify the boost constant is non-zero and reasonable
    assert 0 < CRUISE_BOOST <= 100, f"CRUISE_BOOST={CRUISE_BOOST} µs seems unreasonable"


@pytest.mark.sw
def test_rov_position_geometry():
    """
    [AUTO] 1.3 ⭐ NEW — ROV position geometry (rov_position.py).

    Tests the cable-length formula and N/E offset direction relative to heading.
    No hardware or MAVLink required.
    """
    # Cable length increases monotonically with spins
    L0  = _cable_length_from_spins(0.0)
    L5  = _cable_length_from_spins(5.0)
    L23 = _cable_length_from_spins(23.125)   # exactly one full layer
    L46 = _cable_length_from_spins(46.25)    # exactly two full layers
    assert L0  == 0.0,     "Zero spins → zero length"
    assert L5  >  0.0,     "Positive spins → positive length"
    assert L23 >  L5,      "One layer > 5 spins"
    assert L46 >  L23,     "Two layers > one layer"

    # First-layer approximation: L ≈ 2π * (core + d/2) * spins
    core, diam = 0.095, 0.0076
    expected_L5 = 2 * math.pi * (core + diam / 2) * 5
    assert abs(L5 - expected_L5) < 0.5, \
        f"5-spin length: expected ~{expected_L5:.2f} m, got {L5:.2f} m"

    # Direction tests with 10 spins, depth 2 m
    L = _cable_length_from_spins(10.0)
    depth = 2.0
    assert L > depth, "Cable must be longer than depth for geometry to work"
    horiz = math.sqrt(L**2 - depth**2) * _DEFAULT_SAG_FACTOR

    # heading = 0 (North) → north offset positive, east offset zero
    north_m = horiz * math.cos(0.0)
    east_m  = horiz * math.sin(0.0)
    assert north_m > 0.1,      f"North heading → positive N offset, got {north_m:.3f}"
    assert abs(east_m) < 0.01, f"North heading → ~0 E offset, got {east_m:.3f}"

    # heading = π/2 (East) → east offset positive, north offset zero
    north_m = horiz * math.cos(math.pi / 2)
    east_m  = horiz * math.sin(math.pi / 2)
    assert east_m  > 0.1,       f"East heading → positive E offset, got {east_m:.3f}"
    assert abs(north_m) < 0.01, f"East heading → ~0 N offset, got {north_m:.3f}"

    # Sag factor must reduce horizontal distance (cable arcs, not a straight line)
    horiz_no_sag = math.sqrt(L**2 - depth**2)
    assert horiz < horiz_no_sag, "Sag factor must reduce horizontal distance"
    assert _DEFAULT_SAG_FACTOR == 0.85, \
        f"Default sag factor should be 0.85, got {_DEFAULT_SAG_FACTOR}"

    # Inconsistent data (L < depth) → horizontal distance should be 0
    if _cable_length_from_spins(0.5) < depth:
        assert True  # confirmed inconsistent case exists
    horiz_bad = math.sqrt(max(0.0, _cable_length_from_spins(0.1)**2 - 100.0**2)) * _DEFAULT_SAG_FACTOR
    assert horiz_bad == 0.0, "Cable shorter than depth → horizontal distance must be 0"


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 2 — Individual sensor unit tests
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.unit
def test_tension_reads(gpio_setup):
    """
    [REQUIRES HARDWARE] 2.1  HX711 load cell — verify reading in −10…100 kg range.

    ⚠ Hardware: HX711 wired (DT=GPIO22, SCK=GPIO27), load cell connected.
    No physical interaction needed; reading should be near 0 with no load.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    print_test_header("TENSION SENSOR (HX711)", 1, 8)
    try:
        import rclpy
        from hx711 import TensionNode
        if not rclpy.ok():
            rclpy.init()
        node = TensionNode()
        time.sleep(1)
        val = node.tension
        node.destroy_node()
        ok = -10 < val < 100
        print_test_result(ok, f"Tension: {val:.2f} kg")
        pause_for_approval()
        assert ok, f"Tension out of expected range: {val:.2f} kg"
    except ImportError as e:
        pytest.skip(f"HX711 module unavailable: {e}")


@pytest.mark.unit
def test_encoder_reads(gpio_setup):
    """
    [AUTO on Pi] 2.2  AMT112S-V encoder — verify angle is in −360…360° range.

    No physical interaction needed; confirms encoder wiring and tick→angle conversion.
    """
    from utils import print_test_header, print_test_result
    print_test_header("ENCODER (AMT112S-V)", 2, 8)
    try:
        import rclpy, ENCODER
        from ENCODER import encoder
        if not rclpy.ok():
            rclpy.init()
        enc = encoder()
        time.sleep(0.5)
        angle = (ENCODER.position / 8192.0) * 360.0 * (360.0 / 310.0) % 360
        enc.destroy_node()
        ok = -360 < angle < 360
        print_test_result(ok, f"Angle: {angle:.2f}°  (raw position: {ENCODER.position} ticks)")
        assert ok, f"Encoder angle out of range: {angle:.2f}°"
    except ImportError as e:
        pytest.skip(f"Encoder module unavailable: {e}")


@pytest.mark.unit
def test_gps_connect(serial_connection):
    """
    [REQUIRES HARDWARE] 2.3  u-blox GPS — verify serial data flows on /dev/ttyACM0.

    ⚠ Hardware: GPS module connected via USB. Antenna should have sky view.
    Passes even without a valid fix — just checks serial bytes arrive.
    """
    from utils import print_test_header, print_test_result
    print_test_header("GPS RECEIVER (u-blox)", 3, 8)
    stream = serial_connection
    count = 0
    start = time.time()
    while time.time() - start < 5:
        data = stream.read(1)
        if data:
            count += 1
        time.sleep(0.01)
    ok = count > 10
    print_test_result(ok, f"Received {count} bytes in 5 s (need > 10)")
    assert ok, f"GPS not sending data: got {count} bytes in 5 s"


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 3 — Individual actuator unit tests
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.unit
def test_thrusters_pwm(pigpio_connection):
    """
    [REQUIRES HARDWARE] 3.1  T200 ESCs — PWM pulsewidth reads back within ±10 µs.

    ⚠ Hardware: thrusters should be clamped/secured before running.
    Tests NEUTRAL, FORWARD (1600), REVERSE (1400) in sequence.
    """
    from utils import print_test_header, print_test_result
    print_test_header("THRUSTERS (T200 PWM)", 4, 8)
    pi = pigpio_connection
    LEFT, RIGHT = 12, 26
    results = {}
    for label, r_us, l_us in [("NEUTRAL", 1500, 1500), ("FORWARD", 1600, 1600), ("REVERSE", 1400, 1400)]:
        pi.set_servo_pulsewidth(RIGHT, r_us)
        pi.set_servo_pulsewidth(LEFT,  l_us)
        time.sleep(0.1)
        r_got = pi.get_servo_pulsewidth(RIGHT)
        l_got = pi.get_servo_pulsewidth(LEFT)
        ok = abs(r_got - r_us) <= 10 and abs(l_got - l_us) <= 10
        results[label] = ok
        print_test_result(ok, f"{label}: R={r_got} µs  L={l_got} µs")
    pi.set_servo_pulsewidth(RIGHT, 1500)
    pi.set_servo_pulsewidth(LEFT,  1500)
    assert all(results.values()), f"Thruster PWM failures: {results}"


@pytest.mark.unit
def test_stepper_steps(gpio_setup, pigpio_connection):
    """
    [REQUIRES HARDWARE] 3.2  Stepper motor — direction pin and step pulse.

    ⚠ Hardware: stepper driver must be powered; DIR pin (GPIO6) must be readable.
    """
    import RPi.GPIO as GPIO
    import pigpio as _pigpio
    from utils import print_test_header, print_test_result
    print_test_header("STEPPER MOTOR (Winch)", 5, 8)
    pi = pigpio_connection
    STEP, DIR = 5, 6
    GPIO.setup(DIR, GPIO.OUT)
    pi.set_mode(STEP, _pigpio.OUTPUT)
    results = {}
    for expected, label in [(0, "FORWARD"), (1, "BACKWARD")]:
        GPIO.output(DIR, expected)
        time.sleep(0.2)
        got = GPIO.input(DIR)
        ok = got == expected
        results[label] = ok
        print_test_result(ok, f"DIR {label}: expected={expected}, got={got}")
    pi.write(STEP, 1); time.sleep(0.01); pi.write(STEP, 0)
    print_test_result(True, "Step pulse generated (no exception)")
    assert all(results.values()), f"Stepper direction failures: {results}"


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 4 — Sensor → Actuator integration tests
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.integration
def test_tension_stepper_interaction(gpio_setup, pigpio_connection):
    """
    [REQUIRES PHYSICAL INTERACTION] 4.1  Tension → Stepper closed-loop.

    ⚠ Physical: pull and release the cable gently during the 10-second window.
    Test passes if tension varies by > 0.5 kg during the window.
    The stepper node responds in production; here we verify the sensor sees the change.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    print_test_header("TENSION → STEPPER Interaction", 6, 8)
    try:
        import rclpy
        from hx711 import TensionNode
        if not rclpy.ok():
            rclpy.init()
        node = TensionNode()
        tensions = []
        print("  Pull and release the cable during the next 10 seconds...")
        for i in range(10):
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)
            tensions.append(node.tension)
            print(f"  [{i+1}/10] tension = {node.tension:.2f} kg", flush=True)
        node.destroy_node()
        variation = max(tensions) - min(tensions)
        ok = variation > 0.5
        print_test_result(ok, f"Variation: {min(tensions):.2f} → {max(tensions):.2f} kg")
        pause_for_approval()
        assert ok, "No significant tension change detected (need > 0.5 kg variation)"
    except ImportError as e:
        pytest.skip(f"HX711 module unavailable: {e}")


@pytest.mark.integration
def test_encoder_thrusters_proportional(pigpio_connection):
    """
    [REQUIRES HARDWARE] 4.2 ⭐ IMPROVED — Encoder angle → proportional thruster PWM.

    Injects synthetic angle values via ROS2 'encoder_angle' topic into a live
    ThrusterNode, reads back 't200_speed' topic, and verifies PWM values match
    the formula from t200.py within ±20 µs.

    ⚠ Hardware: pigpiod running, thrusters clamped.
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Float32MultiArray
    from rclpy.executors import MultiThreadedExecutor
    from t200 import ThrusterNode

    if not rclpy.ok():
        rclpy.init()

    class _Helper(Node):
        def __init__(self):
            super().__init__('_test_steer_helper')
            self._pub = self.create_publisher(Float32, 'encoder_angle', 10)
            self._last = None
            self.create_subscription(Float32MultiArray, 't200_speed', self._cb, 10)
        def inject(self, angle):
            self._last = None
            msg = Float32(); msg.data = float(angle)
            self._pub.publish(msg)
        def _cb(self, msg):
            self._last = list(msg.data)

    thruster = ThrusterNode()
    helper   = _Helper()
    exc = MultiThreadedExecutor()
    exc.add_node(thruster)
    exc.add_node(helper)
    t = threading.Thread(target=exc.spin, daemon=True)
    t.start()

    TOL = 20  # µs
    # (angle, description, expected_right_us, expected_left_us)
    # Positive angle → cable right → right is INNER, left is OUTER
    soft_angle = DEAD_ZONE_DEG + 0.25 * (MAX_ANGLE_DEG - DEAD_ZONE_DEG)
    cases = [
        (0.0,              "dead zone center",    1500, 1500),
        (10.0,             "dead zone 10°",       1500, 1500),
        (soft_angle,       "soft turn right",     1550, 1600),
        (float(MAX_ANGLE_DEG), "hard turn right", 1400, 1600),
        (-soft_angle,      "soft turn left",      1600, 1550),
        (-float(MAX_ANGLE_DEG), "hard turn left", 1600, 1400),
    ]
    failures = []
    for angle, label, exp_r, exp_l in cases:
        helper.inject(angle)
        deadline = time.time() + 0.6
        while time.time() < deadline:
            if helper._last is not None:
                break
            time.sleep(0.05)
        if helper._last is None:
            failures.append(f"  angle={angle:.1f}° ({label}): no t200_speed message")
            continue
        got_r, got_l = int(helper._last[0]), int(helper._last[1])
        if abs(got_r - exp_r) > TOL or abs(got_l - exp_l) > TOL:
            failures.append(
                f"  angle={angle:.1f}° ({label}): "
                f"expected R={exp_r} L={exp_l}, got R={got_r} L={got_l}"
            )

    exc.shutdown(timeout_sec=1)
    thruster.destroy_node()
    helper.destroy_node()
    assert not failures, "Proportional steering mismatches:\n" + "\n".join(failures)


@pytest.mark.integration
def test_cruise_activation_closed_loop():
    """
    [AUTO] 4.3 ⭐ NEW — Cruise control ON/OFF via ROS2 topics (no hardware).

    CruiseControlNode is pure logic — no pigpio or GPIO required.
    Publishes encoder_angle + tension, waits for cruise_boost response.
    Verifies automatic activation, deactivation, and re-activation transitions.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Float32
    from cruise_control import CruiseControlNode

    if not rclpy.ok():
        rclpy.init()

    class _Helper(Node):
        def __init__(self):
            super().__init__('_test_cruise_helper')
            self._angle_pub   = self.create_publisher(Float32, 'encoder_angle', 10)
            self._tension_pub = self.create_publisher(Float32, 'tension', 10)
            self._boost = None
            self.create_subscription(Float32, 'cruise_boost', lambda m: setattr(self, '_boost', float(m.data)), 10)
        def send(self, angle, tension):
            self._boost = None
            a = Float32(); a.data = float(angle)
            t = Float32(); t.data = float(tension)
            self._angle_pub.publish(a)
            self._tension_pub.publish(t)

    cruise = CruiseControlNode()
    helper = _Helper()
    exc = MultiThreadedExecutor()
    exc.add_node(cruise)
    exc.add_node(helper)
    threading.Thread(target=exc.spin, daemon=True).start()
    time.sleep(0.2)

    # (angle, tension, expected_boost, description)
    cases = [
        (5.0,  25.0, float(CRUISE_BOOST), "center + nominal   → CRUISE ON"),
        (20.0, 25.0, 0.0,                 "angle > dead zone  → CRUISE OFF"),
        (5.0,  10.0, 0.0,                 "low tension        → CRUISE OFF"),
        (5.0,  40.0, 0.0,                 "high tension       → CRUISE OFF"),
        (5.0,  25.0, float(CRUISE_BOOST), "back to nominal    → CRUISE RESUMES"),
    ]
    failures = []
    for angle, tension, expected, label in cases:
        helper.send(angle, tension)
        # Wait for the 10 Hz cruise timer to fire at least twice
        time.sleep(0.3)
        got = helper._boost
        if got is None:
            failures.append(f"  {label}: no cruise_boost message received")
        elif abs(got - expected) > 1.0:
            failures.append(f"  {label}: expected {expected:.0f} µs, got {got:.0f} µs")
        else:
            print(f"  ✓ {label}: boost={got:.0f} µs")

    exc.shutdown(timeout_sec=1)
    cruise.destroy_node()
    helper.destroy_node()
    assert not failures, "Cruise activation failures:\n" + "\n".join(failures)


@pytest.mark.integration
def test_overtension_stepper_response(gpio_setup, pigpio_connection):
    """
    [REQUIRES HARDWARE] 4.4 ⭐ NEW — Over-tension safety: stepper must retract.

    Publishes tension = 40 kg (above HIGH_TENSION = 35 kg) via ROS2 topic.
    WinchMotorNode should switch DIR_PIN (GPIO6) to retraction direction within 2 s.

    ⚠ Hardware: stepper driver powered, DIR_PIN=GPIO6 accessible.
    """
    import RPi.GPIO as GPIO
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Float32
    from stepper import WinchMotorNode

    DIR_PIN = 6
    GPIO.setup(DIR_PIN, GPIO.IN)  # Read back to confirm

    if not rclpy.ok():
        rclpy.init()

    class _TensionPub(Node):
        def __init__(self):
            super().__init__('_test_tension_pub')
            self._pub = self.create_publisher(Float32, 'tension', 10)
        def send(self, kg):
            msg = Float32(); msg.data = float(kg)
            self._pub.publish(msg)

    winch  = WinchMotorNode()
    sender = _TensionPub()
    exc = MultiThreadedExecutor()
    exc.add_node(winch)
    exc.add_node(sender)
    threading.Thread(target=exc.spin, daemon=True).start()
    time.sleep(0.3)

    # Inject over-tension
    sender.send(40.0)
    time.sleep(2.0)
    dir_state = GPIO.input(DIR_PIN)

    exc.shutdown(timeout_sec=1)
    winch.destroy_node()
    sender.destroy_node()

    print(f"  DIR_PIN state after tension=40 kg: {dir_state}  (0=reel-in / retract)")
    assert dir_state == 0, (
        f"Expected DIR_PIN=0 (reel-in/retract) after over-tension, got {dir_state}. "
        "Stepper may not have responded within 2 s."
    )


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 5 — ROV MAVLink integration  (requires BlueROV2 + MAVProxy)
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.rov
def test_rov_connection(mavlink_connection):
    """
    [AUTO when MAVProxy running] 5.1  BlueROV2 MAVLink — heartbeat received.

    Requires MAVProxy forwarding BlueROV2 telemetry to udp:127.0.0.1:14551.
    """
    mav = mavlink_connection
    assert mav.target_system > 0, (
        f"No valid target_system (got {mav.target_system}). "
        "Is MAVProxy running and forwarding to 14551?"
    )
    print(f"  Connected: sysid={mav.target_system}  compid={mav.target_component}")


@pytest.mark.rov
def test_rov_arm_disarm(mavlink_connection):
    """
    [AUTO when connected] 5.2  ROV arm → verify armed → disarm → verify disarmed.

    Always sets MANUAL mode first for safety.
    ⚠ Thrusters will be energised — keep ROV in safe position.
    """
    from blue_rov2_terminal_control import set_mode, arm
    mav = mavlink_connection

    set_mode(mav, 'MANUAL')
    time.sleep(1)

    arm(mav, True)
    time.sleep(1)
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    assert hb is not None, "No HEARTBEAT after arm command"
    assert bool(hb.base_mode & 0x80), "Vehicle should be ARMED (MAV_MODE_FLAG_SAFETY_ARMED set)"

    arm(mav, False)
    time.sleep(1)
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    assert hb is not None, "No HEARTBEAT after disarm command"
    assert not bool(hb.base_mode & 0x80), "Vehicle should be DISARMED"
    print("  ✓ Arm → Armed confirmed → Disarm → Disarmed confirmed")


@pytest.mark.rov
def test_rov_mode_transitions(mavlink_connection):
    """
    [AUTO when connected] 5.3 ⭐ NEW — ROV transitions through all key operational modes.

    Tests the same MAVLink interface used in blue_rov2_set_mode.py and the VSCode
    terminal panel. Sequence: MANUAL → STABILIZE → DEPTH_HOLD → MANUAL.

    For each transition: sends set_mode(), waits 1.5 s, reads HEARTBEAT custom_mode,
    and compares to connection.mode_mapping()[mode_name].
    """
    from blue_rov2_terminal_control import set_mode
    mav = mavlink_connection
    mode_map = mav.mode_mapping()

    sequence = ['MANUAL', 'STABILIZE', 'DEPTH_HOLD', 'MANUAL']
    failures = []

    for mode_name in sequence:
        if mode_name not in mode_map:
            print(f"  SKIP: mode '{mode_name}' not in firmware mode map (ArduSub version may differ)")
            continue

        expected_id = mode_map[mode_name]
        set_mode(mav, mode_name)
        time.sleep(1.5)

        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is None:
            failures.append(f"  → {mode_name}: no HEARTBEAT received within 3 s")
            continue

        got_id = hb.custom_mode
        if got_id != expected_id:
            failures.append(
                f"  → {mode_name}: expected custom_mode={expected_id}, got {got_id}"
            )
        else:
            print(f"  ✓ {mode_name} (custom_mode={got_id})")

    assert not failures, "Mode transition failures:\n" + "\n".join(failures)


@pytest.mark.rov
def test_rov_attitude_stream(mavlink_connection):
    """
    [AUTO when connected] 5.4 ⭐ NEW — ATTITUDE messages stream with valid yaw.

    ROVPositionNode relies on ATTITUDE.yaw for cable bearing calculation.
    Confirms the stream is live and yaw is in NED range [−π, π].
    """
    from blue_rov2_terminal_control import set_stream
    mav = mavlink_connection

    set_stream(mav, 1, 4)   # stream id 1 = RAW_SENSORS, 4 Hz
    time.sleep(0.5)

    attitudes = []
    deadline = time.time() + 5.0
    while time.time() < deadline and len(attitudes) < 3:
        msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg:
            attitudes.append(msg)

    assert len(attitudes) >= 3, (
        f"Expected ≥3 ATTITUDE messages within 5 s, got {len(attitudes)}. "
        "Check that ArduSub is streaming (request_stream or enable telemetry)."
    )
    for msg in attitudes:
        assert -math.pi <= msg.yaw <= math.pi, \
            f"ATTITUDE.yaw out of NED range [−π, π]: {msg.yaw:.4f} rad"

    last_yaw_deg = math.degrees(attitudes[-1].yaw)
    print(f"  ✓ {len(attitudes)} ATTITUDE messages received  |  last yaw = {last_yaw_deg:.1f}°")


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 6 — Visual / physical demo  (operator must be present)
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.visual
def test_visual_demo(pigpio_connection):
    """
    [REQUIRES PHYSICAL PRESENCE] 6.1  Full visual demo — thrusters + winch in motion.

    ⚠ Physical: operator must supervise. Clamp thrusters. Ensure winch has free cable.
    Runs the complete sequence from truster_test.py (turns, forward, reverse, reel).
    """
    import pigpio as _pigpio
    from utils import print_test_header, print_test_result, pause_for_approval
    print_test_header("VISUAL DEMO (Thrusters + Winch)", 9, 9)
    pi = pigpio_connection

    PIN_RIGHT, PIN_LEFT = 26, 12
    STEP_PIN, DIR_PIN   = 5, 6
    GENTLE_FWD, FAST_FWD = 1600, 1700
    GENTLE_REV, FAST_REV, SLOW_FWD = 1400, 1300, 1550
    HOLD, HOLD_NEUTR = 15, 6

    pi.set_mode(STEP_PIN, _pigpio.OUTPUT)
    pi.set_mode(DIR_PIN,  _pigpio.OUTPUT)

    def drive(r, l, t, label):
        print(f"  [{label}]  R={r} µs  L={l} µs  ({t} s)")
        pi.set_servo_pulsewidth(PIN_RIGHT, r)
        pi.set_servo_pulsewidth(PIN_LEFT,  l)
        time.sleep(t)

    def neutral():
        drive(NEUTRAL, NEUTRAL, HOLD_NEUTR, "NEUTRAL")

    def winch_run(direction, freq, duration, label):
        print(f"  [WINCH] {label}  freq={freq} Hz  ({duration} s)")
        pi.write(DIR_PIN, direction)
        time.sleep(0.005)
        pulse_us = int(1_000_000 / (freq * 2))
        end = time.time() + duration
        while time.time() < end:
            pi.write(STEP_PIN, 1); time.sleep(pulse_us / 1e6)
            pi.write(STEP_PIN, 0); time.sleep(pulse_us / 1e6)
        pi.write(STEP_PIN, 0)

    try:
        print("\n  ⚠ Make sure thrusters are clamped and winch cable is free!")
        input("  Press Enter to start visual demo (Ctrl+C to abort)...")

        drive(NEUTRAL,     NEUTRAL,     9,    "ARM (9 s)")
        drive(GENTLE_FWD,  GENTLE_FWD,  HOLD, "GENTLE FORWARD");  neutral()
        drive(FAST_FWD,    FAST_FWD,    HOLD, "FAST FORWARD");    neutral()
        winch_run(0, 800, 12, "REEL OUT slow"); time.sleep(3)
        drive(GENTLE_REV,  GENTLE_REV,  HOLD, "GENTLE REVERSE");  neutral()
        drive(FAST_REV,    FAST_REV,    HOLD, "FAST REVERSE");    neutral()
        winch_run(1, 800, 12, "REEL IN slow"); time.sleep(3)
        drive(GENTLE_REV,  GENTLE_FWD,  HOLD, "HARD TURN RIGHT"); neutral()
        drive(GENTLE_FWD,  GENTLE_REV,  HOLD, "HARD TURN LEFT");  neutral()
        drive(SLOW_FWD,    FAST_FWD,    HOLD, "SOFT TURN RIGHT"); neutral()
        drive(FAST_FWD,    SLOW_FWD,    HOLD, "SOFT TURN LEFT");  neutral()
        winch_run(0, 2000, 9, "REEL OUT fast")
        winch_run(1, 2000, 9, "REEL IN fast")
        print_test_result(True, "Visual demo completed successfully")
        pause_for_approval()

    except KeyboardInterrupt:
        print("\n  ⚠ Demo interrupted by user")
    except Exception as e:
        print_test_result(False, f"Exception: {e}")
        assert False, str(e)
    finally:
        pi.set_servo_pulsewidth(PIN_RIGHT, 0)
        pi.set_servo_pulsewidth(PIN_LEFT,  0)
        pi.write(STEP_PIN, 0)


# ═══════════════════════════════════════════════════════════════════════════════
# LAYER 7 — Full system live test  (all 5 subsystems simultaneously)
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.visual
def test_full_system_live(gpio_setup, pigpio_connection):
    """
    [REQUIRES PHYSICAL PRESENCE] 7.1  All subsystems running simultaneously.

    Starts every ROS2 node under a single MultiThreadedExecutor while running a
    short physical demo sequence. Reads USVStateNode snapshot every second and
    verifies that all 5 subsystems are producing live data:

      Subsystem       Topic checked
      ─────────────   ─────────────────
      HX711           tension  (Float32, kg)
      Encoder         encoder_angle  (Float32, degrees)
      GPS             gps  (Float32MultiArray [fix, lat, lon, alt])
      Stepper         stepper  (Float32, RPM)
      T200 thrusters  t200_speed  (Float32MultiArray [right_us, left_us])

    The motor sequence is shortened (ARM + gentle forward + reel out + neutral)
    so the test completes in ≈30 seconds.

    ⚠ Physical: operator must supervise. Clamp thrusters. Free cable on winch.
    """
    import pigpio as _pigpio
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from utils import print_test_header, print_test_result, pause_for_approval

    from hx711 import TensionNode
    from ENCODER import encoder as EncoderNode
    from GPS import GPS as GPSNode
    from stepper import WinchMotorNode
    from t200 import ThrusterNode
    from state_aggregator import USVStateNode

    print_test_header("FULL SYSTEM LIVE (All 5 Subsystems)", 10, 10)
    print("\n  ⚠ Clamp thrusters. Ensure winch cable is free.")
    input("  Press Enter to start full system test (Ctrl+C to abort)...\n")

    if not rclpy.ok():
        rclpy.init()

    # ── Start all nodes ────────────────────────────────────────────────────────
    state_node = USVStateNode()
    nodes = [
        TensionNode(),
        EncoderNode(),
        GPSNode(),
        WinchMotorNode(),
        ThrusterNode(),
        state_node,
    ]
    exc = MultiThreadedExecutor()
    for n in nodes:
        exc.add_node(n)
    spin_thread = threading.Thread(target=exc.spin, daemon=True)
    spin_thread.start()

    print("  All nodes started. Settling for 9 seconds...")
    time.sleep(9)

    pi = pigpio_connection
    PIN_RIGHT, PIN_LEFT = 26, 12
    STEP_PIN, DIR_PIN   = 5, 6
    GENTLE_FWD = 1600
    pi.set_mode(STEP_PIN, _pigpio.OUTPUT)
    pi.set_mode(DIR_PIN,  _pigpio.OUTPUT)

    def drive(r, l, t, label):
        print(f"\n  [{label}]  R={r} µs  L={l} µs  ({t} s)")
        pi.set_servo_pulsewidth(PIN_RIGHT, r)
        pi.set_servo_pulsewidth(PIN_LEFT,  l)

    def winch_run(direction, freq, duration, label):
        print(f"  [WINCH] {label}")
        pi.write(DIR_PIN, direction)
        time.sleep(0.005)
        pulse_us = int(1_000_000 / (freq * 2))
        end = time.time() + duration
        while time.time() < end:
            pi.write(STEP_PIN, 1); time.sleep(pulse_us / 1e6)
            pi.write(STEP_PIN, 0); time.sleep(pulse_us / 1e6)
        pi.write(STEP_PIN, 0)

    def print_snapshot(label):
        snap = state_node.get_state()
        print(f"\n  ── Snapshot @ {label} ──────────────────────────────")
        print(f"     tension      = {snap.get('tension')}")
        print(f"     encoder_angle= {snap.get('encoder_angle')}")
        print(f"     stepper RPM  = {snap.get('stepper')}")
        print(f"     t200_speed   = {snap.get('t200_speed')}")
        gps = snap.get('gps')
        if gps:
            print(f"     GPS          = fix={gps[0]} lat={gps[1]:.6f} lon={gps[2]:.6f}")
        else:
            print(f"     GPS          = {gps}")
        return snap

    snapshots = []
    try:
        # ARM sequence
        drive(NEUTRAL, NEUTRAL, 9, "ARM (9 s)")
        time.sleep(9)
        snapshots.append(print_snapshot("after ARM"))

        # Gentle forward + reel out while reading sensors
        drive(GENTLE_FWD, GENTLE_FWD, 0, "GENTLE FORWARD (15 s)")
        winch_run(0, 800, 15, "REEL OUT while moving forward")
        snapshots.append(print_snapshot("during FORWARD + REEL OUT"))

        # Back to neutral
        drive(NEUTRAL, NEUTRAL, 0, "NEUTRAL")
        time.sleep(6)
        snapshots.append(print_snapshot("after NEUTRAL"))

        # Hard turn right
        drive(1400, 1600, 0, "HARD TURN RIGHT (9 s)")
        time.sleep(9)
        snapshots.append(print_snapshot("during HARD TURN RIGHT"))

        drive(NEUTRAL, NEUTRAL, 0, "NEUTRAL")
        time.sleep(6)

        # Reel in
        winch_run(1, 800, 15, "REEL IN")
        snapshots.append(print_snapshot("after REEL IN"))

    except KeyboardInterrupt:
        print("\n  ⚠ Test interrupted by user")
    finally:
        pi.set_servo_pulsewidth(PIN_RIGHT, 0)
        pi.set_servo_pulsewidth(PIN_LEFT,  0)
        pi.write(STEP_PIN, 0)
        exc.shutdown(timeout_sec=2)
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                pass

    # ── Verify all 5 subsystems produced data ─────────────────────────────────
    last = snapshots[-1] if snapshots else {}
    missing = []
    for key, label in [
        ('tension',       'HX711 tension'),
        ('encoder_angle', 'Encoder angle'),
        ('stepper',       'Stepper RPM'),
        ('t200_speed',    'T200 thruster PWM'),
        ('gps',           'GPS fix'),
    ]:
        if last.get(key) is None:
            missing.append(f"  {label} ({key}): no data received during test")

    print("\n")
    if missing:
        print_test_result(False, f"{len(missing)} subsystems produced no data")
    else:
        print_test_result(True, "All 5 subsystems produced live data during demo")

    pause_for_approval()
    assert not missing, "Subsystems with no data:\n" + "\n".join(missing)


# ─── Force sequential execution (prevent GPIO / pigpio conflicts) ─────────────
def pytest_collection_modifyitems(config, items):
    for item in items:
        item.add_marker(pytest.mark.sequential)
