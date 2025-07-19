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

# ====== TỐC ĐỘ ĐIỀU CHỈNH DUY NHẤT ======
BASE_SPEED = 25  # ✅ CHỈ CẦN THAY ĐỔI GIÁ TRỊ NÀY

# ====== PID CẢI TIẾN ======
class AdvancedPID:
    def __init__(self, kp=2.0, ki=0.1, kd=0.5, integral_limit=10.0, output_limit=50.0):
        # Tham số PID
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        
        # Giới hạn
        self.integral_limit = integral_limit  # Giới hạn tích phân (chống windup)
        self.output_limit = output_limit      # Giới hạn đầu ra
        
        # Biến trạng thái
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()
        
        # Bộ lọc nhiễu cho derivative
        self.prev_derivative = 0
        self.derivative_filter = 0.3  # Hệ số lọc (0-1, càng nhỏ càng mượt)
        
        # Adaptive gain - tự động điều chỉnh
        self.base_kp = kp
        self.error_history = []
        self.history_size = 10
        
    def compute(self, error):
        now = time.time()
        dt = now - self.prev_time if now > self.prev_time else 0.01
        self.prev_time = now
        
        # === PROPORTIONAL ===
        # Adaptive Kp - tăng Kp khi error lớn để phản ứng nhanh hơn
        if abs(error) > 0.5:
            adaptive_kp = self.base_kp * 1.5  # Tăng 50% khi lệch nhiều
        else:
            adaptive_kp = self.base_kp
            
        proportional = adaptive_kp * error
        
        # === INTEGRAL với chống windup ===
        self.integral += error * dt
        
        # Giới hạn integral để tránh windup
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
            
        # Reset integral khi error đổi dấu (qua zero crossing)
        if self.prev_error * error < 0:
            self.integral *= 0.5  # Giảm một nửa thay vì reset hoàn toàn
            
        integral_term = self.ki * self.integral
        
        # === DERIVATIVE với lọc nhiễu ===
        raw_derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        # Lọc derivative để giảm nhiễu
        filtered_derivative = (self.derivative_filter * raw_derivative + 
                             (1 - self.derivative_filter) * self.prev_derivative)
        self.prev_derivative = filtered_derivative
        
        derivative_term = self.kd * filtered_derivative
        
        # === TỔNG HỢP ===
        output = proportional + integral_term + derivative_term
        
        # Giới hạn output
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
            
        # Lưu lại cho lần tính tiếp theo
        self.prev_error = error
        
        # Debug thông tin chi tiết
        print(f"PID Debug: P={proportional:.2f}, I={integral_term:.2f}, D={derivative_term:.2f}, Out={output:.2f}")
        
        return output
    
    def reset(self):
        """Reset PID khi cần khởi tạo lại"""
        self.integral = 0
        self.prev_error = 0
        self.prev_derivative = 0
        self.prev_time = time.time()
        
    def tune_auto(self, error_history):
        """Tự động điều chỉnh PID dựa trên lịch sử error"""
        if len(error_history) < 20:
            return
            
        # Tính độ dao động
        error_variance = sum([(e - sum(error_history)/len(error_history))**2 for e in error_history]) / len(error_history)
        
        # Nếu dao động nhiều → giảm Kd, tăng Ki
        if error_variance > 0.3:
            self.kd *= 0.95
            self.ki *= 1.02
        # Nếu phản ứng chậm → tăng Kp  
        elif abs(sum(error_history[-5:])/5) > 0.2:
            self.kp *= 1.05

# ====== ĐIỀU KHIỂN ĐỘNG CƠ ======
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    
    # Bảo vệ giá trị PWM
    speed_l = max(0, min(100, abs(speed_l)))
    speed_r = max(0, min(100, abs(speed_r)))
    
    pwm_ena.ChangeDutyCycle(speed_l)
    pwm_enb.ChangeDutyCycle(speed_r)

def move_forward(speed_l, speed_r):
    """Di chuyển tiến với tốc độ có thể khác nhau cho 2 bánh"""
    set_motor(0, 1, 1, 0, speed_l, speed_r)

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

# ====== CẢM BIẾN LINE ======
def read_sensors():
    return (GPIO.input(SENSOR_LEFT), GPIO.input(SENSOR_CENTER), GPIO.input(SENSOR_RIGHT))

# ====== TÌM LẠI LINE ======
def search_for_line(direction=1):
    speed = BASE_SPEED
    if direction == 1:  # Tìm về bên trái
        set_motor(1, 0, 0, 1, speed, speed)  # Quay trái
    else:  # Tìm về bên phải  
        set_motor(0, 1, 1, 0, speed, speed)  # Quay phải

# ====== DÒ LINE CHÍNH VỚI PID CẢI TIẾN ======
def line_following():
    L, C, R = read_sensors()
    print(f"Cảm biến: L={L}, C={C}, R={R}")
    
    # Xác định error với độ chính xác cao hơn
    if C == 1 and L == 0 and R == 0:
        # Line ở giữa - hoàn hảo
        error = 0
    elif L == 1 and C == 1 and R == 0:
        # Line lệch nhẹ về trái
        error = -0.3
    elif L == 0 and C == 1 and R == 1:
        # Line lệch nhẹ về phải  
        error = 0.3
    elif L == 1 and C == 0 and R == 0:
        # Line lệch nhiều về trái
        error = -1.0
    elif L == 0 and C == 0 and R == 1:
        # Line lệch nhiều về phải
        error = 1.0
    elif L == 1 and R == 1:
        # Giao lộ hoặc line rộng - đi thẳng
        error = 0
    elif C == 1 and L == 1 and R == 1:
        # Line rất rộng - đi thẳng
        error = 0
    else:
        # Mất line hoàn toàn
        print(">> Mất line, đang tìm...")
        search_for_line(direction=1)
        return
    
    # PID điều khiển với thuật toán cải tiến
    correction = pid.compute(error)
    
    # Tính tốc độ với giới hạn an toàn
    left_speed = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction
    
    # Đảm bảo tốc độ không âm và không quá cao
    left_speed = max(0, min(80, left_speed))
    right_speed = max(0, min(80, right_speed))
    
    # Debug info chi tiết
    print(f"Error: {error}, Correction: {correction:.2f}, L_Speed: {left_speed:.1f}, R_Speed: {right_speed:.1f}")
    
    # Di chuyển với tốc độ đã điều chỉnh
    move_forward(left_speed, right_speed)

# ====== KẾT THÚC ======
def cleanup():
    stop()
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã thoát và dọn dẹp GPIO")

atexit.register(cleanup)

# ====== CHẠY CHƯƠNG TRÌNH VỚI PID CẢI TIẾN ======
pid = AdvancedPID(kp=2.0, ki=0.1, kd=0.5, integral_limit=10.0, output_limit=50.0)
error_history = []  # Để auto-tuning

print("🚗 Bắt đầu dò line với PID cải tiến...")

try:
    loop_count = 0
    while True:
        line_following()
        
        # Auto-tuning mỗi 100 vòng lặp
        loop_count += 1
        if loop_count % 100 == 0:
            pid.tune_auto(error_history)
            error_history = []  # Reset history
            
        time.sleep(0.01)
        
except KeyboardInterrupt:
    cleanup()