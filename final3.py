import RPi.GPIO as GPIO
import time

# ====== CHÂN GPIO ======
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

MOTOR_PINS = {
    'IN1': 6,   # Trái
    'IN2': 5,
    'ENA': 12,
    'IN3': 22,  # Phải
    'IN4': 27,
    'ENB': 13
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 500)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 500)
pwm_ena.start(0)
pwm_enb.start(0)

def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    
    speed_l = max(0, min(100, abs(speed_l)))
    speed_r = max(0, min(100, abs(speed_r)))
    
    pwm_ena.ChangeDutyCycle(speed_l)
    pwm_enb.ChangeDutyCycle(speed_r)

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

def debug_sensors():
    """Test cảm biến"""
    print("=== TEST CẢM BIẾN ===")
    print("Đặt robot trên line và di chuyển để test...")
    
    for i in range(50):
        L = GPIO.input(SENSOR_LEFT)
        C = GPIO.input(SENSOR_CENTER) 
        R = GPIO.input(SENSOR_RIGHT)
        print(f"L={L} C={C} R={R}")
        time.sleep(0.2)

def debug_motors():
    """Test từng động cơ"""
    print("=== TEST ĐỘNG CƠ ===")
    
    print("1. Test động cơ TRÁI tiến...")
    set_motor(0, 1, 0, 0, 50, 0)
    time.sleep(2)
    stop()
    time.sleep(1)
    
    print("2. Test động cơ TRÁI lùi...")
    set_motor(1, 0, 0, 0, 50, 0)
    time.sleep(2)
    stop()
    time.sleep(1)
    
    print("3. Test động cơ PHẢI tiến...")
    set_motor(0, 0, 1, 0, 0, 50)
    time.sleep(2)
    stop()
    time.sleep(1)
    
    print("4. Test động cơ PHẢI lùi...")
    set_motor(0, 0, 0, 1, 0, 50)
    time.sleep(2)
    stop()
    time.sleep(1)
    
    print("5. Test cả 2 động cơ tiến...")
    set_motor(0, 1, 1, 0, 50, 50)
    time.sleep(2)
    stop()

def test_turning():
    """Test khả năng rẽ"""
    print("=== TEST RẼ ===")
    
    print("1. Rẽ TRÁI (bánh phải nhanh hơn)...")
    set_motor(0, 1, 1, 0, 20, 50)  # Trái chậm, phải nhanh
    time.sleep(2)
    stop()
    time.sleep(1)
    
    print("2. Rẽ PHẢI (bánh trái nhanh hơn)...")
    set_motor(0, 1, 1, 0, 50, 20)  # Trái nhanh, phải chậm
    time.sleep(2)
    stop()

def test_line_response():
    """Test phản ứng với line"""
    print("=== TEST PHẢN ỨNG LINE ===")
    print("Đặt robot trên line và quan sát...")
    
    for i in range(20):
        L = GPIO.input(SENSOR_LEFT)
        C = GPIO.input(SENSOR_CENTER)
        R = GPIO.input(SENSOR_RIGHT)
        
        print(f"\nCảm biến: L={L}, C={C}, R={R}")
        
        if L == 1 and C == 0 and R == 0:
            print("→ Line ở TRÁI, robot cần RẼ TRÁI")
            print("→ Giảm tốc độ bánh TRÁI, tăng tốc độ bánh PHẢI")
            set_motor(0, 1, 1, 0, 20, 50)
            
        elif L == 0 and C == 0 and R == 1:
            print("→ Line ở PHẢI, robot cần RẼ PHẢI") 
            print("→ Tăng tốc độ bánh TRÁI, giảm tốc độ bánh PHẢI")
            set_motor(0, 1, 1, 0, 50, 20)
            
        elif C == 1:
            print("→ Line ở GIỮA, robot đi THẲNG")
            set_motor(0, 1, 1, 0, 40, 40)
            
        else:
            print("→ Không thấy line")
            stop()
            
        time.sleep(1)
        stop()
        time.sleep(0.5)

def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("\nĐã thoát và dọn dẹp GPIO")

# ====== MENU CHÍNH ======
def main():
    try:
        while True:
            print("\n" + "="*40)
            print("🔧 ROBOT DEBUG TOOL")
            print("="*40)
            print("1. Test cảm biến")
            print("2. Test động cơ") 
            print("3. Test khả năng rẽ")
            print("4. Test phản ứng line")
            print("0. Thoát")
            
            choice = input("\nChọn test (0-4): ")
            
            if choice == "1":
                debug_sensors()
            elif choice == "2":
                debug_motors()
            elif choice == "3":
                test_turning()
            elif choice == "4":
                test_line_response()
            elif choice == "0":
                break
            else:
                print("Lựa chọn không hợp lệ!")
                
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()

if __name__ == "__main__":
    main()