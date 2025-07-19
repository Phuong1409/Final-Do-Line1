import RPi.GPIO as GPIO
import time
import atexit

# ====== CHÂN GPIO ======
# Cảm biến line
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

# Động cơ
MOTOR_PINS = {
    'IN1': 6,   # Trái
    'IN2': 5,
    'ENA': 12,
    'IN3': 22,  # Phải
    'IN4': 27,
    'ENB': 13
}

# ====== CÀI ĐẶT CHUNG ======
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

# PWM động cơ
pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 500)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 500)
pwm_ena.start(0)
pwm_enb.start(0)

# ====== TỐC ĐỘ ĐIỀU CHỈNH  ======
BASE_SPEED = 20  

# ====== PID ======
class PID:
    def __init__(self, kp=1.2, ki=0.05, kd=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.prev_time if now > self.prev_time else 0.01
        self.prev_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# ====== ĐIỀU KHIỂN ĐỘNG CƠ ======
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)

    # Bảo vệ giá trị PWM
    speed_l = max(0, min(100, speed_l))
    speed_r = max(0, min(100, speed_r))

    pwm_ena.ChangeDutyCycle(speed_l)
    pwm_enb.ChangeDutyCycle(speed_r)

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

# ====== CẢM BIẾN LINE ======
def read_sensors():
    return (GPIO.input(SENSOR_LEFT), GPIO.input(SENSOR_CENTER), GPIO.input(SENSOR_RIGHT))

# ====== TÌM LẠI LINE ======
def search_for_line(direction=1):
    speed = BASE_SPEED
    if direction == 1:
        set_motor(1, 0, 0, 1, speed, speed)  # quay trái
    else:
        set_motor(0, 1, 1, 0, speed, speed)  # quay phải

# ====== DÒ LINE CHÍNH ======
def line_following():
    L, C, R = read_sensors()
    print(f"Cảm biến: L={L}, C={C}, R={R}")

    if C == 1 and L == 0 and R == 0:
        error = 0
    elif L == 1 and C == 0:
        error = 1
    elif R == 1 and C == 0:
        error = -1
    elif L == 1 and R == 1:
        error = 0
    elif C == 1 and (L == 1 or R == 1):
        error = 0
    else:
        print(">> Mất line, đang tìm...")
        search_for_line(direction=1)
        return

    # PID điều khiển
    correction = pid.compute(error)
    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction

    set_motor(0, 1, 1, 0, left_speed, right_speed)

# ====== KẾT THÚC ======
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã thoát và dọn dẹp GPIO")

atexit.register(cleanup)

# ====== CHẠY CHƯƠNG TRÌNH ======
pid = PID(kp=1.2, ki=0.05, kd=0.2)
print("🚗 Bắt đầu dò line...")
try:
    while True:
        line_following()
        time.sleep(0.01)
except KeyboardInterrupt:
    cleanup()
