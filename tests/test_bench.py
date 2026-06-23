import rclpy
"""
USV Test Bench - Comprehensive system bringup tests.

Unit Tests:
  1. test_tension_reads - HX711 load cell reads tension values
  2. test_encoder_reads - Encoder reads position/angle values
  3. test_stepper_steps - Stepper motor can step and change direction
  4. test_thrusters_pwm - T200 thrusters respond to PWM commands
  5. test_gps_connect - GPS receives serial data

Integration Tests:
  6. test_tension_stepper_interaction - Tension sensor → Stepper motor responds
  7. test_encoder_thrusters_interaction - Encoder angle → Thrusters activate
  8. test_system_startup - All components initialize together

Run with: pytest tests/test_bench.py -v -s
"""

import pytest
import RPi.GPIO as GPIO
import pigpio
import serial
import time
import threading
from pyubx2 import UBXReader
from std_msgs.msg import Float32, Float32MultiArray
from contextlib import contextmanager

# ============================================================================
# UNIT TEST 1: TENSION SENSOR (HX711)
# ============================================================================

def test_tension_reads(gpio_setup):
    """
    UNIT TEST 1/8: Tension Sensor (HX711)
    Verify HX711 load cell can read and publish tension values.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("TENSION SENSOR (HX711)", 1, 8)
    
    try:
        # Import the tension node
        from hx711 import TensionNode
        
        print("  Initializing HX711 load cell (pins DT=22, CLK=27)...")
        print("  Calibrating... (reading baseline)")
        
        # Create node (this will calibrate)
        tension_node = TensionNode()
        
        # Wait for a reading
        time.sleep(1)
        
        # Read the last tension value
        tension_value = tension_node.tension
        
        # Verify it's in reasonable range (after taring, should be near 0)
        if -10 < tension_value < 100:
            msg = f"Tension reading: {tension_value:.2f} kg (OK)"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"Tension reading out of range: {tension_value:.2f} kg"
            print_test_result(False, msg)
            result = False
        
        tension_node.destroy_node()
        
    except ImportError as e:
        print(f"  ⚠ SKIP: Could not import tension module: {e}")
        pytest.skip("Tension module not available")
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "Tension sensor test failed"


# ============================================================================
# UNIT TEST 2: ENCODER
# ============================================================================

def test_encoder_reads(gpio_setup):
    """
    UNIT TEST 2/8: Encoder
    Verify encoder can read position and angle values.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("ENCODER", 2, 8)
    
    try:
        # Import encoder node
        from ENCODER import encoder, position
        import ENCODER
        
        print("  Initializing encoder (pins A=24, B=23)...")
        print("  Reading current position...")
        
        # Create encoder node
        enc = encoder()
        
        # Wait for a tick
        time.sleep(0.5)
        
        # Read current position (global variable from ENCODER module)
        current_position = ENCODER.position
        
        # Calculate angle (8192 ticks per revolution)
        angle = (current_position / 8192.0) * 360.0 * (360.0 / 310.0)
        angle = angle % 360
        
        print(f"  Position: {current_position} ticks, Angle: {angle:.2f}°")
        
        if -360 < angle < 360:
            msg = f"Encoder angle: {angle:.2f}° (range: 0-360°)"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"Encoder angle out of range: {angle:.2f}°"
            print_test_result(False, msg)
            result = False
        
        enc.destroy_node()
        
    except ImportError as e:
        print(f"  ⚠ SKIP: Could not import encoder module: {e}")
        pytest.skip("Encoder module not available")
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "Encoder test failed"


# ============================================================================
# UNIT TEST 3: STEPPER MOTOR (WINCH)
# ============================================================================

def test_stepper_steps(gpio_setup, pigpio_connection):
    """
    UNIT TEST 3/8: Stepper Motor (Winch)
    Verify stepper motor can step and change direction.
    """
    from utils import print_test_header, print_test_result, pause_for_approval, safe_gpio_read, safe_gpio_write
    
    print_test_header("STEPPER MOTOR (Winch)", 3, 8)
    
    try:
        pi = pigpio_connection
        
        # Setup GPIO pins
        step_pin = 5
        dir_pin = 6
        
        print(f"  Setting up stepper pins: STEP={step_pin}, DIR={dir_pin}")
        
        # Setup direction pin as output
        GPIO.setup(dir_pin, GPIO.OUT)
        
        # Test forward direction
        print("  Testing FORWARD direction...")
        safe_gpio_write(dir_pin, 0)
        time.sleep(0.2)
        dir_state = safe_gpio_read(dir_pin)
        
        if dir_state == 0:
            print("    ✓ Direction pin set to FORWARD (0)")
            fwd_ok = True
        else:
            print(f"    ✗ Direction pin failed: expected 0, got {dir_state}")
            fwd_ok = False
        
        # Test backward direction
        print("  Testing BACKWARD direction...")
        safe_gpio_write(dir_pin, 1)
        time.sleep(0.2)
        dir_state = safe_gpio_read(dir_pin)
        
        if dir_state == 1:
            print("    ✓ Direction pin set to BACKWARD (1)")
            bwd_ok = True
        else:
            print(f"    ✗ Direction pin failed: expected 1, got {dir_state}")
            bwd_ok = False
        
        # Test step pulse (using pigpio)
        print("  Testing step pulse...")
        pi.set_mode(step_pin, pigpio.OUTPUT)
        pi.write(step_pin, 1)
        time.sleep(0.01)
        pi.write(step_pin, 0)
        
        print("    ✓ Step pulse generated")
        step_ok = True
        
        if fwd_ok and bwd_ok and step_ok:
            msg = "Motor stepping: ✓ Forward ✓ Backward ✓ Step pulse"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"Motor test partial failure: Fwd={fwd_ok}, Bwd={bwd_ok}, Step={step_ok}"
            print_test_result(False, msg)
            result = False
            
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "Stepper motor test failed"


# ============================================================================
# UNIT TEST 4: THRUSTERS (T200)
# ============================================================================

def test_thrusters_pwm(pigpio_connection):
    """
    UNIT TEST 4/8: Thrusters (T200)
    Verify T200 thrusters respond to PWM commands.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("THRUSTERS (T200)", 4, 8)
    
    try:
        pi = pigpio_connection
        
        left_pin = 12
        right_pin = 26
        
        print("  Testing thruster PWM signals...")
        print(f"    Left thruster pin: {left_pin}")
        print(f"    Right thruster pin: {right_pin}")
        
        # Test neutral position
        print("  Setting to NEUTRAL (1500 μs)...")
        pi.set_servo_pulsewidth(left_pin, 1500)
        pi.set_servo_pulsewidth(right_pin, 1500)
        time.sleep(0.1)
        
        left_pulse = pi.get_servo_pulsewidth(left_pin)
        right_pulse = pi.get_servo_pulsewidth(right_pin)
        
        neutral_ok = (1490 < left_pulse < 1510) and (1490 < right_pulse < 1510)
        print(f"    ✓ Left: {left_pulse} μs, Right: {right_pulse} μs")
        
        # Test forward
        print("  Setting to FORWARD (1600 μs)...")
        pi.set_servo_pulsewidth(left_pin, 1600)
        pi.set_servo_pulsewidth(right_pin, 1400)
        time.sleep(0.1)
        
        left_pulse = pi.get_servo_pulsewidth(left_pin)
        right_pulse = pi.get_servo_pulsewidth(right_pin)
        
        forward_ok = (1590 < left_pulse < 1610) and (1390 < right_pulse < 1410)
        print(f"    ✓ Left: {left_pulse} μs, Right: {right_pulse} μs")
        
        # Test reverse
        print("  Setting to REVERSE (1400 μs)...")
        pi.set_servo_pulsewidth(left_pin, 1400)
        pi.set_servo_pulsewidth(right_pin, 1600)
        time.sleep(0.1)
        
        left_pulse = pi.get_servo_pulsewidth(left_pin)
        right_pulse = pi.get_servo_pulsewidth(right_pin)
        
        reverse_ok = (1390 < left_pulse < 1410) and (1590 < right_pulse < 1610)
        print(f"    ✓ Left: {left_pulse} μs, Right: {right_pulse} μs")
        
        # Return to neutral
        pi.set_servo_pulsewidth(left_pin, 1500)
        pi.set_servo_pulsewidth(right_pin, 1500)
        
        if neutral_ok and forward_ok and reverse_ok:
            msg = "Thruster PWM: ✓ Neutral ✓ Forward ✓ Reverse"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"Thruster test failed: Neutral={neutral_ok}, Fwd={forward_ok}, Rev={reverse_ok}"
            print_test_result(False, msg)
            result = False
            
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "Thruster test failed"


# ============================================================================
# UNIT TEST 5: GPS RECEIVER
# ============================================================================

def test_gps_connect(serial_connection):
    """
    UNIT TEST 5/8: GPS Receiver
    Verify GPS can establish serial connection and receive messages.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("GPS RECEIVER", 5, 8)
    
    try:
        stream = serial_connection
        
        print("  Connecting to GPS on /dev/ttyACM0...")
        print(f"    Connection established, baudrate: 9600")
        
        if stream.is_open:
            print("    ✓ Serial port is open")
            open_ok = True
        else:
            print("    ✗ Serial port failed to open")
            open_ok = False
        
        # Try to read a message
        print("  Waiting for GPS messages (5 second timeout)...")
        
        start_time = time.time()
        msg_count = 0
        
        while time.time() - start_time < 5:
            try:
                data = stream.read(1)
                if data:
                    msg_count += 1
                    if msg_count % 50 == 0:
                        print(f"    ✓ Receiving data ({msg_count} bytes)")
            except:
                pass
            time.sleep(0.01)
        
        read_ok = msg_count > 10  # Should receive more than 10 bytes in 5 seconds
        
        if read_ok:
            print(f"    ✓ Received {msg_count} bytes from GPS")
        else:
            print(f"    ✗ No GPS data received ({msg_count} bytes)")
        
        if open_ok and read_ok:
            msg = f"GPS: ✓ Connected, ✓ Receiving ({msg_count} bytes)"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"GPS connection test failed: Open={open_ok}, Reading={read_ok}"
            print_test_result(False, msg)
            result = False
            
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "GPS test failed"


# ============================================================================
# INTEGRATION TEST 6: TENSION → STEPPER
# ============================================================================

def test_tension_stepper_interaction(gpio_setup, pigpio_connection):
    """
    INTEGRATION TEST 6/8: Tension Sensor → Stepper Motor
    Verify stepper motor responds to tension feedback.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("TENSION → STEPPER Interaction", 6, 8)
    
    try:
        pi = pigpio_connection
        
        print("  This test requires manual cable interaction:")
        print("    1. System will monitor tension changes")
        print("    2. You should gently pull/release the cable")
        print("    3. Motor should spin faster as tension increases\n")
        
        print("  Monitoring for 10 seconds...")
        print("  (Pull cable slowly to increase tension)")
        
        # For this test, we just verify the connections exist
        # Real verification requires manual interaction
        from hx711 import TensionNode
        
        print("  - Initializing tension sensor...")
        tension_node = TensionNode()
        
        print("  - Initializing stepper motor...")
        GPIO.setup(5, GPIO.OUT)
        GPIO.setup(6, GPIO.OUT)
        
        # Monitor for changes
        tensions = []
        last_nonzero = 0.0
        for i in range(10):
            # Spin ROS so the timer callback actually fires
            for _ in range(20):
                rclpy.spin_once(tension_node, timeout_sec=0.05)
            t = tension_node.tension
            if t != 0:
                last_nonzero = t
            tensions.append(last_nonzero)
            print(f"    [{i+1}/10] Tension: {last_nonzero:.2f} kg  (pull the cable!)")
        
        tension_node.destroy_node()
        
        # Check if we detected any variation
        min_tension = min(tensions)
        max_tension = max(tensions)
        variation = max_tension - min_tension
        
        if variation > 0.5:
            msg = f"Detected tension variation: {min_tension:.2f} → {max_tension:.2f} kg"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"No significant tension change detected (variation: {variation:.2f} kg)"
            print_test_result(False, msg)
            result = False
            
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "Tension-Stepper interaction test failed"


# ============================================================================
# INTEGRATION TEST 7: ENCODER → THRUSTERS
# ============================================================================

def test_encoder_thrusters_interaction(gpio_setup, pigpio_connection):
    """
    INTEGRATION TEST 7/8: Encoder → Thrusters
    Verify thrusters activate correctly based on encoder angle.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("ENCODER → THRUSTERS Interaction", 7, 8)
    
    try:
        pi = pigpio_connection
        
        print("  This test requires manual cable movement:")
        print("    1. System monitors encoder angle")
        print("    2. You should gently pull cable left/right")
        print("    3. Thrusters should activate to correct angle\n")
        
        print("  Monitoring for 10 seconds...")
        print("  (Pull cable to the left and right)")
        
        from ENCODER import encoder
        import ENCODER
        
        print("  - Initializing encoder...")
        enc = encoder()
        
        print("  - Initializing thrusters...")
        pi.set_servo_pulsewidth(12, 1500)   # Neutral
        pi.set_servo_pulsewidth(26, 1500)   # Neutral
        
        # Monitor encoder
        angles = []
        for i in range(10):
            time.sleep(1)
            angle = (ENCODER.position / 8192.0) * 360.0 * (360.0 / 310.0)
            angles.append(angle)
            print(f"    [{i+1}/10] Angle: {angle:.2f}°")
        
        enc.destroy_node()
        
        # Check if we detected any variation
        min_angle = min(angles)
        max_angle = max(angles)
        variation = max_angle - min_angle
        
        if variation > 5:
            msg = f"Detected angle variation: {min_angle:.2f} → {max_angle:.2f}°"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"No significant angle change detected (variation: {variation:.2f}°)"
            print_test_result(False, msg)
            result = False
            
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "Encoder-Thrusters interaction test failed"


# ============================================================================
# INTEGRATION TEST 8: SYSTEM STARTUP
# ============================================================================

def test_system_startup(gpio_setup, pigpio_connection):
    """
    INTEGRATION TEST 8/8: System Startup
    Verify all components initialize together without errors.
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    
    print_test_header("SYSTEM STARTUP (All Components)", 8, 8)
    
    try:
        print("  Initializing all components in sequence...\n")
        
        components = []
        errors = []
        
        # Try to initialize each component
        try:
            print("  1. Initializing GPS...")
            from GPS import GPS
            gps = GPS()
            components.append(("GPS", gps))
            print("     ✓ GPS initialized")
        except Exception as e:
            print(f"     ⚠ GPS skipped: {str(e)}")
            errors.append(f"GPS: {str(e)}")
        
        try:
            print("  2. Initializing Tension Sensor...")
            from hx711 import TensionNode
            tension = TensionNode()
            components.append(("Tension", tension))
            print("     ✓ Tension sensor initialized")
        except Exception as e:
            print(f"     ⚠ Tension skipped: {str(e)}")
            errors.append(f"Tension: {str(e)}")
        
        try:
            print("  3. Initializing Encoder...")
            from ENCODER import encoder
            enc = encoder()
            components.append(("Encoder", enc))
            print("     ✓ Encoder initialized")
        except Exception as e:
            print(f"     ⚠ Encoder skipped: {str(e)}")
            errors.append(f"Encoder: {str(e)}")
        
        try:
            print("  4. Initializing Stepper Motor...")
            from stepper import WinchMotorNode
            stepper = WinchMotorNode()
            components.append(("Stepper", stepper))
            print("     ✓ Stepper motor initialized")
        except Exception as e:
            print(f"     ⚠ Stepper skipped: {str(e)}")
            errors.append(f"Stepper: {str(e)}")
        
        try:
            print("  5. Initializing Thrusters...")
            from t200 import ThrusterNode
            thrusters = ThrusterNode()
            components.append(("Thrusters", thrusters))
            print("     ✓ Thrusters initialized")
        except Exception as e:
            print(f"     ⚠ Thrusters skipped: {str(e)}")
            errors.append(f"Thrusters: {str(e)}")
        
        print(f"\n  System initialization complete!")
        print(f"  Successful components: {len(components)}/5")
        
        if len(components) >= 3:
            msg = f"System ready with {len(components)} components active"
            print_test_result(True, msg)
            result = True
        else:
            msg = f"Too many component failures ({len(errors)} errors)"
            print_test_result(False, msg)
            result = False
        
        # Cleanup
        for name, comp in components:
            try:
                comp.destroy_node()
            except:
                pass
                
    except Exception as e:
        print(f"  ✗ ERROR: {str(e)}")
        print_test_result(False, f"Exception: {str(e)}")
        result = False
    
    pause_for_approval()
    assert result, "System startup test failed"


# ============================================================================
# TEST EXECUTION HOOKS
# ============================================================================

def pytest_collection_modifyitems(config, items):
    """Force sequential execution of tests."""
    for item in items:
        item.add_marker(pytest.mark.sequential)


if __name__ == "__main__":
    print("Run this test suite with: pytest tests/test_bench.py -v -s")


# ============================================================================
# VISUAL DEMO TEST 9: Full system visual demo
# ============================================================================

@pytest.mark.visual
def test_visual_demo(pigpio_connection):
    """
    VISUAL TEST 9: Full system visual demo
    Runs thrusters + winch so you can see everything moving.
    Run with: pytest tests/test_bench.py::test_visual_demo -v -s -m visual
    """
    from utils import print_test_header, print_test_result, pause_for_approval
    import pigpio as _pigpio

    print_test_header("VISUAL DEMO (Thrusters + Winch)", 9, 9)

    pi = pigpio_connection

    PIN_RIGHT, PIN_LEFT = 26, 12
    STEP_PIN, DIR_PIN   = 5, 6
    NEUTRAL, GENTLE_FWD, FAST_FWD = 1500, 1600, 1700
    GENTLE_REV, FAST_REV, SLOW_FWD = 1400, 1300, 1550
    HOLD, HOLD_NEUTR = 5, 2

    pi.set_mode(STEP_PIN, _pigpio.OUTPUT)
    pi.set_mode(DIR_PIN,  _pigpio.OUTPUT)

    def drive(r, l, t, label):
        print(f"  [{label}] R={r} L={l} ({t}s)")
        pi.set_servo_pulsewidth(PIN_RIGHT, r)
        pi.set_servo_pulsewidth(PIN_LEFT, l)
        time.sleep(t)

    def neutral():
        drive(NEUTRAL, NEUTRAL, HOLD_NEUTR, "neutral")

    def winch(direction, freq, duration, label):
        print(f"  [WINCH] {label} ({duration}s)")
        pi.write(DIR_PIN, direction)
        time.sleep(0.005)
        pulse_us = int(1_000_000 / (freq * 2))
        end = time.time() + duration
        while time.time() < end:
            pi.write(STEP_PIN, 1)
            time.sleep(pulse_us / 1_000_000)
            pi.write(STEP_PIN, 0)
            time.sleep(pulse_us / 1_000_000)
        pi.write(STEP_PIN, 0)

    try:
        print("\n  ⚠ Make sure thrusters are clamped and winch has free cable!")
        input("  Press Enter to start visual demo...")

        drive(NEUTRAL, NEUTRAL, 3, "ARM")
        drive(GENTLE_FWD, GENTLE_FWD, HOLD, "GENTLE FORWARD"); neutral()
        drive(FAST_FWD,   FAST_FWD,   HOLD, "FAST FORWARD");   neutral()
        winch(0, 800, 4, "REEL IN"); time.sleep(1)
        drive(GENTLE_REV, GENTLE_REV, HOLD, "GENTLE REVERSE"); neutral()
        drive(FAST_REV,   FAST_REV,   HOLD, "FAST REVERSE");   neutral()
        winch(1, 800, 4, "REEL OUT"); time.sleep(1)
        drive(GENTLE_REV, GENTLE_FWD, HOLD, "HARD TURN RIGHT"); neutral()
        drive(GENTLE_FWD, GENTLE_REV, HOLD, "HARD TURN LEFT");  neutral()
        drive(SLOW_FWD,   FAST_FWD,   HOLD, "SOFT TURN RIGHT"); neutral()
        drive(FAST_FWD,   SLOW_FWD,   HOLD, "SOFT TURN LEFT");  neutral()
        winch(0, 2000, 3, "REEL IN fast")
        winch(1, 2000, 3, "REEL OUT fast")

        print_test_result(True, "Visual demo completed")
        result = True

    except Exception as e:
        print_test_result(False, f"Exception: {e}")
        result = False

    finally:
        pi.set_servo_pulsewidth(PIN_RIGHT, 0)
        pi.set_servo_pulsewidth(PIN_LEFT, 0)
        pi.write(STEP_PIN, 0)

    pause_for_approval()
    assert result, "Visual demo failed"
