import RPi.GPIO as GPIO
import time
import threading

# ==========================================
# GPIO Pin Definitions (BCM)
# ==========================================
# 1. Stepper motor (iSV57T)
PUL_PIN = 18  # Physical pin 12
DIR_PIN = 25  # Physical pin 22

# 2. Load cell / cable tension sensor (HX711)
DT_PIN  = 22  # Physical pin 15
SCK_PIN = 27  # Physical pin 13

# 3. Position encoder (AMT112S-V)
A_PIN = 24    # Physical pin 18
B_PIN = 23    # Physical pin 16

# ==========================================
# Global variables
# ==========================================
current_tension = 0
encoder_position = 0
keep_reading = True
pwm = None

def setup():
    GPIO.setmode(GPIO.BCM)
    
    # --- Motor settings ---
    GPIO.setup(PUL_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    global pwm
    pwm = GPIO.PWM(PUL_PIN, 200) # Initial frequency
    
    # --- HX711 settings ---
    GPIO.setup(DT_PIN, GPIO.IN)
    GPIO.setup(SCK_PIN, GPIO.OUT)
    GPIO.output(SCK_PIN, False)
    
    # --- Encoder settings (with internal pull-up) ---
    GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Register hardware interrupts for encoder (runs asynchronously in background)
    GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=update_encoder_position)
    GPIO.add_event_detect(B_PIN, GPIO.BOTH, callback=update_encoder_position)

# ==========================================
# Component 1: Encoder logic (interrupt-driven)
# ==========================================
def update_encoder_position(channel):
    global encoder_position
    a = GPIO.input(A_PIN)
    b = GPIO.input(B_PIN)
    if channel == A_PIN:
        if a == b:
            encoder_position += 1
        else:
            encoder_position -= 1
    else:
        if a != b:
            encoder_position += 1
        else:
            encoder_position -= 1

# ==========================================
# Component 2: Load cell logic (HX711) – runs in background thread
# ==========================================
def read_hx711_raw():
    timeout = time.time() + 1.0
    while GPIO.input(DT_PIN) == 1:
        if time.time() > timeout:
            return None
        time.sleep(0.001)
        
    value = 0
    for _ in range(24):
        GPIO.output(SCK_PIN, True)
        value = (value << 1) | GPIO.input(DT_PIN)
        GPIO.output(SCK_PIN, False)
        
    GPIO.output(SCK_PIN, True)
    GPIO.output(SCK_PIN, False)
    
    if value & 0x800000:
        value -= 0x1000000
    return value

def load_cell_monitor():
    global current_tension, keep_reading
    while keep_reading:
        v = read_hx711_raw()
        if v is not None:
            current_tension = v
        time.sleep(0.05)

# ==========================================
# Component 3: Motion control and data display
# ==========================================
def execute_motion(direction, duration):
    """Control motor and display encoder/tension data in real-time."""
    current_freq = 200
    target_freq = 4000
    step = 200
    delay = 0.05
    
    GPIO.output(DIR_PIN, direction)
    pwm.start(50)
    pwm.ChangeFrequency(current_freq)
    
    # 1. Acceleration phase
    while current_freq < target_freq:
        current_freq += step
        pwm.ChangeFrequency(current_freq)
        print(f"Accelerating... | Frequency: {current_freq:4}Hz | Cable tension (Raw): {current_tension:>9} | Encoder position: {encoder_position:>6} ticks", end='\r')
        time.sleep(delay)
        
    # 2. Cruise phase (continuous sampling for specified duration)
    end_time = time.time() + duration
    while time.time() < end_time:
        print(f"Cruising...  | Frequency: {current_freq:4}Hz | Cable tension (Raw): {current_tension:>9} | Encoder position: {encoder_position:>6} ticks", end='\r')
        time.sleep(0.1)
        
    # 3. Deceleration phase
    while current_freq > 200:
        current_freq -= step
        pwm.ChangeFrequency(current_freq)
        print(f"Decelerating... | Frequency: {current_freq:4}Hz | Cable tension (Raw): {current_tension:>9} | Encoder position: {encoder_position:>6} ticks", end='\r')
        time.sleep(delay)
        
    pwm.stop()
    print(f"\nTemporary stop executed. Temporary final position: {encoder_position} ticks")

def main():
    global keep_reading
    try:
        print("=== Starting integrated system test ===")
        print("Initialize sensors and reset systems...")
        
        # Start HX711 background thread
        sensor_thread = threading.Thread(target=load_cell_monitor)
        sensor_thread.setDaemon(True) # Ensures thread closes on shutdown
        sensor_thread.start()
        
        time.sleep(0.5) # Stabilization time
        
        # Run direction 1 (quick 5-second safe test)
        print("\n>>> Running motion direction 1 (HIGH) for 5 seconds <<<")
        execute_motion(GPIO.HIGH, 5)
        
        print("\nWaiting 2 seconds...")
        time.sleep(2)
        
        # Run direction 2 (reverse)
        print("\n>>> Running motion direction 2 (LOW) for 5 seconds <<<")
        execute_motion(GPIO.LOW, 5)
        
        print("\n[✓] Integrated test completed successfully!")
        print(f"Final encoder position: {encoder_position} ticks")

    except KeyboardInterrupt:
        print("\n[!] Emergency stop triggered by user!")
        if pwm:
            pwm.stop()
    finally:
        keep_reading = False
        time.sleep(0.2)
        GPIO.cleanup()
        print("GPIO cleaned up successfully. System secured.")

if __name__ == '__main__':
    setup()
    main()