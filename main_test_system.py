import RPi.GPIO as GPIO
import time
import threading

# ==========================================
# הגדרות פינים (BCM)
# ==========================================
# 1. מנוע כננת (iSV57T)
PUL_PIN = 18  # פין פיזי 12
DIR_PIN = 25  # פין פיזי 22

# 2. חיישן עומס / מתיחות כבל (HX711)
DT_PIN  = 22  # פין פיזי 15
SCK_PIN = 27  # פין פיזי 13

# 3. אנקודר מיקום (AMT112S-V)
A_PIN = 24    # פין פיזי 18
B_PIN = 23    # פין פיזי 16

# ==========================================
# משתנים גלובליים
# ==========================================
current_tension = 0
encoder_position = 0
keep_reading = True
pwm = None

def setup():
    GPIO.setmode(GPIO.BCM)
    
    # --- הגדרות מנוע ---
    GPIO.setup(PUL_PIN, GPIO.OUT)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    global pwm
    pwm = GPIO.PWM(PUL_PIN, 200) # תדר התחלתי
    
    # --- הגדרות HX711 ---
    GPIO.setup(DT_PIN, GPIO.IN)
    GPIO.setup(SCK_PIN, GPIO.OUT)
    GPIO.output(SCK_PIN, False)
    
    # --- הגדרות אנקודר (כולל Pull-up פנימי) ---
    GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # רישום פסיקות חומרה לאנקודר (עובד ברקע בצורה אסינכרונית)
    GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=update_encoder_position)
    GPIO.add_event_detect(B_PIN, GPIO.BOTH, callback=update_encoder_position)

# ==========================================
# רכיב 1: לוגיקת האנקודר (מבוסס פסיקות)
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
# רכיב 2: לוגיקת תא עומס (HX711) - רץ בתהליכון רקע
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
# רכיב 3: בקרת תנועה (חזית) ותצוגת נתונים
# ==========================================
def execute_motion(direction, duration):
    """ מפעיל את המנוע ומציג את נתוני האנקודר והמתיחות בזמן אמת """
    current_freq = 200
    target_freq = 4000
    step = 200
    delay = 0.05
    
    GPIO.output(DIR_PIN, direction)
    pwm.start(50)
    pwm.ChangeFrequency(current_freq)
    
    # 1. שלב ההאצה
    while current_freq < target_freq:
        current_freq += step
        pwm.ChangeFrequency(current_freq)
        print(f"מאיץ...  | תדר: {current_freq:4}Hz | מתח כבל (Raw): {current_tension:>9} | מיקום אנקודר: {encoder_position:>6} ticks", end='\r')
        time.sleep(delay)
        
    # 2. שלב השיוט (דגימה רציפה למשך הזמן שהוגדר)
    end_time = time.time() + duration
    while time.time() < end_time:
        print(f"שיוט...  | תדר: {current_freq:4}Hz | מתח כבל (Raw): {current_tension:>9} | מיקום אנקודר: {encoder_position:>6} ticks", end='\r')
        time.sleep(0.1)
        
    # 3. שלב ההאטה
    while current_freq > 200:
        current_freq -= step
        pwm.ChangeFrequency(current_freq)
        print(f"מאט...   | תדר: {current_freq:4}Hz | מתח כבל (Raw): {current_tension:>9} | מיקום אנקודר: {encoder_position:>6} ticks", end='\r')
        time.sleep(delay)
        
    pwm.stop()
    print(f"\nעצירה זמנית בוצעה. מיקום סופי זמני: {encoder_position} ticks")

def main():
    global keep_reading
    try:
        print("=== תחילת טסט מערכת משולב מלא ===")
        print("מפעיל חיישנים ומאפס מערכות...")
        
        # הפעלת תהליכון ה-HX711 ברקע
        sensor_thread = threading.Thread(target=load_cell_monitor)
        sensor_thread.setDaemon(True) # מבטיח שהתהליכון ייסגר אם התוכנית הראשית קורסת
        sensor_thread.start()
        
        time.sleep(0.5) # זמן התייצבות
        
        # הרצת כיוון 1 לשנייה (לשם בדיקה מהירה ובטוחה של 5 שניות)
        print("\n>>> מריץ תנועה בכיוון 1 (HIGH) ל-5 שניות <<<")
        execute_motion(GPIO.HIGH, 5)
        
        print("\nהמתנה של 2 שניות...")
        time.sleep(2)
        
        # הרצת כיוון 2 (חזרה)
        print("\n>>> מריץ תנועה בכיוון 2 (LOW) ל-5 שניות <<<")
        execute_motion(GPIO.LOW, 5)
        
        print("\n[V] הטסט המשולב הסתיים בהצלחה!")
        print(f"מיקום אנקודר סופי בהחלט: {encoder_position} ticks")

    except KeyboardInterrupt:
        print("\n[!] עצירת חירום הופעלה על ידי המשתמש!")
        if pwm:
            pwm.stop()
    finally:
        keep_reading = False
        time.sleep(0.2)
        GPIO.cleanup()
        print("GPIO נוקה בהצלחה. מערכת מאובטחת.")

if __name__ == '__main__':
    setup()
    main()