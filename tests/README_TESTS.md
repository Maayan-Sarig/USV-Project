# USV Test Bench

Comprehensive system bringup test suite for the Unmanned Surface Vehicle.

## Overview

The test bench contains 8 sequential tests:

### Unit Tests (Test individual components)
1. **Tension Sensor (HX711)** - Reads cable tension with load cell
2. **Encoder** - Reads cable angle/position from rotary encoder
3. **Stepper Motor (Winch)** - Tests motor stepping and direction control
4. **Thrusters (T200)** - Tests PWM servo signals for left/right thrusters
5. **GPS Receiver** - Tests serial communication with u-blox GPS module

### Integration Tests (Test components working together)
6. **Tension → Stepper** - Verifies stepper motor responds to tension feedback
7. **Encoder → Thrusters** - Verifies thrusters correct cable angle
8. **System Startup** - Verifies all components initialize together

## Running Tests

### Prerequisites
- Raspberry Pi with GPIO access
- ROS 2 environment sourced
- Hardware connected:
  - HX711 load cell (pins 22, 27)
  - Encoder (pins 23, 24)
  - Stepper motor (pins 5, 6)
  - T200 thrusters (pins 12, 26) via pigpio
  - GPS on /dev/ttyACM0
- `pigpiod` daemon running: `sudo pigpiod`

### Run Full Test Suite
```bash
cd /home/lar/USV
pytest tests/test_bench.py -v -s
```

### Run Specific Test
```bash
pytest tests/test_bench.py::test_tension_reads -v -s
pytest tests/test_bench.py::test_system_startup -v -s
```

### Options
- `-v` : Verbose output
- `-s` : Show print statements (don't capture output)
- `--tb=short` : Shorter error traceback format

## Operator Experience

After each test, the system will display:
```
[1/8] Unit Test: TENSION SENSOR (HX711)
  Reading current tension value...
  ✓ PASS - Tension: 2.3 kg (expected: 0-100 kg)

Press Enter to continue...
```

You can:
- **Press Enter** to continue to the next test
- **Press Ctrl+C** to abort the test suite

## Integration Tests

The integration tests require manual interaction:

### Test 6: Tension → Stepper
- Gently pull and release the cable
- Watch motor frequency increase/decrease with tension
- Stepper should respond smoothly to tension changes

### Test 7: Encoder → Thrusters  
- Pull cable left and right
- Thrusters should activate to center the cable
- Right pull → left thruster activates
- Left pull → right thruster activates

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `pigpiod daemon not running` | Run `sudo pigpiod` in another terminal |
| GPS serial port not found | Check `/dev/ttyACM0` exists: `ls /dev/ttyACM0` |
| GPIO permission denied | Add user to GPIO groups or run as root |
| Module import errors | Ensure ROS 2 is sourced: `source /opt/ros/<distro>/setup.bash` |

## Test Files Structure

```
tests/
├── __init__.py          # Package marker
├── conftest.py          # Pytest fixtures and utilities
├── test_bench.py        # All 8 tests in one file
└── README_TESTS.md      # This file
```

## Original Tests (Unchanged)

The original test files remain in the root directory:
- `main_test_system.py` - Full system integration test
- `test_gps.py` - GPS module test
- `truster_test.py` - Thruster PWM test
- `Tester.py` - ROS2 experiment harness

## Design Decisions

- **Single File**: All tests in `test_bench.py` for simplicity
- **Sequential Execution**: Tests run one at a time to prevent GPIO/pigpio conflicts
- **Interactive Approval**: Each test pauses for operator confirmation
- **Real Hardware Only**: No mocking - tests verify actual hardware
- **Focused Scope**: Tests verify "does it work?" not edge cases
- **Safety First**: Motor timeouts and neutral commands after each test
