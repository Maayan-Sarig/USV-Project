import RPi.GPIO as GPIO
import rclpy.logging

def quiet_node(node):
    """Suppress a ROS2 node's INFO/WARN console spam (e.g. '[INFO] [thruster_node]: ...',
    repeated GPS read-error warnings) so only test-relevant output shows. Errors/fatals
    still print, since those usually do matter."""
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    return node

def print_test_header(test_name, test_number, total_tests):
    print(f"\n{'='*50}")
    print(f"[{test_number}/{total_tests}] {test_name}")
    print(f"{'='*50}")

def print_test_result(result, message=""):
    if result:
        print(f"✓ PASS - {message}")
    else:
        print(f"✗ FAIL - {message}")

def pause_for_approval():
    try:
        input("\nPress Enter to continue...")
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        raise

def safe_gpio_read(pin):
    try:
        return GPIO.input(pin)
    except Exception as e:
        print(f"  Error reading GPIO pin {pin}: {e}")
        return None

def safe_gpio_write(pin, value):
    try:
        GPIO.output(pin, value)
        return True
    except Exception as e:
        print(f"  Error writing to GPIO pin {pin}: {e}")
        return False
