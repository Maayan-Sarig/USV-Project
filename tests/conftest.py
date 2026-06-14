"""
Pytest configuration and fixtures for USV test suite.
Handles hardware initialization, cleanup, and test utilities.
"""

import pytest
import RPi.GPIO as GPIO
import pigpio
import serial
import time
from contextlib import contextmanager


# ============================================================================
# PYTEST FIXTURES - Hardware Initialization & Cleanup
# ============================================================================

@pytest.fixture(scope="function")
def gpio_setup():
    """Initialize GPIO for tests."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    yield
    # Cleanup after test
    GPIO.cleanup()


@pytest.fixture(scope="function")
def pigpio_connection():
    """Initialize pigpio daemon connection for thruster/stepper tests."""
    pi = pigpio.pi()
    if not pi.connected:
        pytest.skip("pigpiod daemon not running")
    yield pi
    # Cleanup - set all pins to neutral
    pi.set_servo_pulsewidth(26, 1500)  # Right thruster neutral
    pi.set_servo_pulsewidth(12, 1500)  # Left thruster neutral
    pi.set_servo_pulsewidth(5, 0)      # Stepper step off
    pi.stop()


@pytest.fixture(scope="function")
def serial_connection():
    """Try to establish GPS serial connection."""
    try:
        stream = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
        yield stream
        stream.close()
    except serial.SerialException:
        pytest.skip("GPS serial port /dev/ttyACM0 not available")


# ============================================================================
# UTILITY FUNCTIONS FOR TESTS
# ============================================================================

def print_test_header(test_name, test_number, total_tests):
    """Print formatted test header."""
    print(f"\n{'='*50}")
    print(f"[{test_number}/{total_tests}] {test_name}")
    print(f"{'='*50}")


def print_test_result(result, message=""):
    """Print colored test result."""
    if result:
        print(f"✓ PASS - {message}")
    else:
        print(f"✗ FAIL - {message}")


def pause_for_approval():
    """Wait for user to approve before continuing."""
    try:
        input("\nPress Enter to continue...")
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        raise


def safe_gpio_read(pin):
    """Safely read a GPIO pin."""
    try:
        return GPIO.input(pin)
    except Exception as e:
        print(f"  Error reading GPIO pin {pin}: {e}")
        return None


def safe_gpio_write(pin, value):
    """Safely write to a GPIO pin."""
    try:
        GPIO.output(pin, value)
        return True
    except Exception as e:
        print(f"  Error writing to GPIO pin {pin}: {e}")
        return False


# ============================================================================
# ADDITIONAL FIXTURES for test_comprehensive.py
# ============================================================================

@pytest.fixture(scope="session")
def mavlink_connection():
    """
    Session-scoped MAVLink connection to BlueROV2 via MAVProxy.
    Tests marked @pytest.mark.rov are skipped when this fixture cannot connect.
    Prerequisite: MAVProxy forwarding BlueROV2 telemetry to udp:127.0.0.1:14551.
    """
    try:
        from blue_rov2_terminal_control import connect
        mav = connect('udp:127.0.0.1:14551', timeout=5)
        yield mav
    except Exception as e:
        pytest.skip(f"BlueROV2 not reachable (MAVProxy not running?): {e}")


# ============================================================================
# SESSION-LEVEL SETUP
# ============================================================================

def pytest_configure(config):
    """Configure pytest - print welcome message."""
    print("\n")
    print("=" * 50)
    print("      USV TEST BENCH - System Bringup")
    print("=" * 50)
    print("\nStarting comprehensive system tests...")
    print("Each test will pause for approval before continuing.\n")


def pytest_sessionfinish(session, exitstatus):
    """Print final summary after all tests."""
    print("\n" + "=" * 50)
    print("                  TEST COMPLETE")
    print("=" * 50)
