"""
Pytest configuration and fixtures for USV test suite.
Handles hardware initialization, cleanup, and test utilities.
"""

import pytest
import RPi.GPIO as GPIO
import pigpio
import serial
import subprocess
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
    # Stop stepper waveform first, then set all pins to safe state
    pi.wave_tx_stop()
    pi.wave_clear()
    pi.write(5, 0)                     # Stepper step pin low
    pi.set_servo_pulsewidth(26, 1500)  # Right thruster neutral
    pi.set_servo_pulsewidth(12, 1500)  # Left thruster neutral
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
    Session-scoped MAVLink connection to BlueROV2.
    Tests marked @pytest.mark.rov are skipped when this fixture cannot connect.
    Prerequisite: the ROV's MAVLink stream must be sent to this Pi's IP on
    port 14551 (e.g. a BlueOS MAVLink Endpoint, or MAVProxy --out, pointed at
    this Pi). Listening on 0.0.0.0 (not 127.0.0.1) so it can receive that
    traffic over the network, not just from this machine.
    """
    mav = None
    try:
        from blue_rov2_terminal_control import connect
        mav = connect('udpin:0.0.0.0:14551', timeout=15)
        yield mav
    except Exception as e:
        pytest.skip(f"BlueROV2 not reachable (MAVProxy not running?): {e}")
    finally:
        if mav is not None:
            try:
                mav.close()
            except Exception:
                pass


# ============================================================================
# SESSION-LEVEL SETUP
# ============================================================================

def kill_conflicting_background_processes():
    """
    Kill background processes known to fight this test suite for the same
    hardware — usv_remote_server.py and the socat relay both open
    /dev/ttyACM0 directly, which causes the GPS test's "multiple access on
    port" failures if either is left running. Also force-frees /dev/ttyACM0
    via fuser as a catch-all, in case something else is holding it.
    Patterns are specific (not bare "python"/"pytest") so this can never
    match the current test process itself.
    """
    for pattern in ('usv_remote_server.py', 'socat.*ttyACM0'):
        try:
            subprocess.run(['pkill', '-f', pattern], check=False,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass  # pkill not installed — nothing we can do here
    try:
        subprocess.run(['fuser', '-k', '/dev/ttyACM0'], check=False,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pass  # fuser not installed (psmisc) — nothing we can do here
    time.sleep(1)  # let the kernel actually release the port


def pytest_configure(config):
    """Configure pytest - clean up conflicting processes, print welcome message."""
    kill_conflicting_background_processes()
    print("\n")
    print("=" * 50)
    print("      USV TEST BENCH - System Bringup")
    print("=" * 50)
    print("\nStarting comprehensive system tests...")
    print("Each test will pause for approval before continuing.\n")


def pytest_keyboard_interrupt(excinfo):
    """Emergency stop all hardware on Ctrl+C."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.wave_tx_stop()
            pi.wave_clear()
            pi.write(5, 0)                     # Stepper step pin low
            pi.set_servo_pulsewidth(26, 1500)  # Right thruster neutral
            pi.set_servo_pulsewidth(12, 1500)  # Left thruster neutral
            pi.stop()
    except Exception:
        pass
    try:
        import RPi.GPIO as GPIO
        GPIO.cleanup()
    except Exception:
        pass
    print("\n[!] Ctrl+C — all hardware stopped.")


def pytest_sessionfinish(session, exitstatus):
    """Clean up conflicting processes, then print final summary after all tests."""
    kill_conflicting_background_processes()
    print("\n" + "=" * 50)
    print("                  TEST COMPLETE")
    print("=" * 50)
