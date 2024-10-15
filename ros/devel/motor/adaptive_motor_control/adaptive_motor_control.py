from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import time
from sshkeyboard import listen_keyboard
import Jetson.GPIO as GPIO
import logging

# I2C 버스 설정
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 500  # PCA9685 주파수 설정

# PID 제어 클래스 정의
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# PWMThrottleHat 클래스 정의
class PWMThrottleHat:
    def __init__(self, pwm, channel, pid):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 500
        self.pid = pid

    def set_throttle(self, throttle):
        try:
            pid_output = self.pid.update(throttle)
            pulse = int(0xFFFF * abs(pid_output))
            pulse = min(max(0, pulse), 0xFFFF)  # Ensure pulse is within the valid range
            if pid_output > 0: # 전진
                self.pwm.channels[self.channel + 5].duty_cycle = pulse
                self.pwm.channels[self.channel + 4].duty_cycle = 0
                self.pwm.channels[self.channel + 3].duty_cycle = 0xFFFF
            elif pid_output < 0: # 후진
                self.pwm.channels[self.channel + 5].duty_cycle = pulse
                self.pwm.channels[self.channel + 4].duty_cycle = 0xFFFF
                self.pwm.channels[self.channel + 3].duty_cycle = 0
            else: # 정지
                self.pwm.channels[self.channel + 5].duty_cycle = 0
                self.pwm.channels[self.channel + 4].duty_cycle = 0
                self.pwm.channels[self.channel + 3].duty_cycle = 0
                
        except Exception as e:
            logging.error(f"PWM throttle ERROR: {e}")

# 서보 모터용 PID 제어 클래스 정의
class PIDServoController:
    def __init__(self, servo, pid, channel):
        self.servo = servo
        self.pid = pid
        self.channel = channel

    def set_angle(self, angle):
        pid_output = self.pid.update(angle)
        new_angle = min(max(0, self.servo[self.channel].angle + pid_output), 180)
        self.servo[self.channel].angle = new_angle

# PID 제어기 생성
pid_motor = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=0.5)  # 원하는 속도(setpoint)로 설정
pid_servo = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=100)  # 원하는 각도(setpoint)로 설정

# PWMThrottleHat 인스턴스 생성
motor_hat = PWMThrottleHat(pca, channel=0, pid=pid_motor)

# 서보 모터 제어 설정
kit = ServoKit(channels=16, i2c=i2c, address=0x60)
servo_controller = PIDServoController(kit.servo, pid_servo, channel=0)

# 자율주행 모드 변수
autonomous_mode = False
pan = 100  # pan 변수를 초기화

def turn_left_back():
    global pan
    print("Turning left_back")
    motor_hat.set_throttle(-0.5)  # 좌회전 시 속도 감소
    pan = max(30, pan - 30)
    servo_controller.set_angle(pan)
    time.sleep(2)  # 2초 동안 회전
    motor_hat.set_throttle(0)
    pan = 100
    servo_controller.set_angle(pan)

def turn_right_back():
    global pan
    print("Turning right_back")
    motor_hat.set_throttle(-0.5)  # 좌회전 시 속도 감소
    pan = min(150, pan + 30)
    servo_controller.set_angle(pan)
    time.sleep(2)  # 2초 동안 회전
    motor_hat.set_throttle(0)
    pan = 100
    servo_controller.set_angle(pan)

def turn_left():
    global pan
    print("Turning left")
    motor_hat.set_throttle(0.5)  # 좌회전 시 속도 감소
    pan = max(30, pan - 30)
    servo_controller.set_angle(pan)
    time.sleep(2)  # 2초 동안 회전
    motor_hat.set_throttle(0)
    pan = 100
    servo_controller.set_angle(pan)

def turn_right():
    global pan
    print("Turning right")
    motor_hat.set_throttle(0.5)  # 우회전 시 속도 감소
    pan = min(150, pan + 30)
    servo_controller.set_angle(pan)
    time.sleep(2)  # 2초 동안 회전
    motor_hat.set_throttle(0)
    pan = 100
    servo_controller.set_angle(pan)

def autonomous_drive():
    global autonomous_mode
    while autonomous_mode:
        distance = measure_distance()
        print(f"Measured Distance = {distance:.1f} cm")
        
        if distance < 30:  # 30cm 이내에 장애물이 있으면 멈춤
            print("Obstacle detected! Stopping.")
            motor_hat.set_throttle(0)
        else:
            motor_hat.set_throttle(0.7)  # 전진
        time.sleep(0.1)  # 거리 측정을 주기적으로 수행
        turn_left()
        time.sleep(2)
        turn_right()
        time.sleep(2)

def press(key):
    global pan, autonomous_mode
    if key == 'w':
        print("Motor forward")
        motor_hat.set_throttle(1.0)
    elif key == 's':
        print("Motor backward")
        motor_hat.set_throttle(-1.0)
    elif key == 'a':
        print("Servo left")
        turn_left()
    elif key == 'd':
        print("Servo right")
        turn_right()
    elif key == 'z': # 후진 좌회전
        print("Servo back_left")
        turn_left_back()
    elif key == 'c': # 후진 우회전
        print("Servo back_right")
        turn_right_back()
    elif key == 'm':
        autonomous_mode = not autonomous_mode
        if autonomous_mode:
            print("Autonomous mode activated")
            autonomous_drive()
        else:
            print("Manual mode activated")
    elif key == 't':  # 정지
        print("Motor stopped")
        motor_hat.set_throttle(0)

def release(key):
    global autonomous_mode
    if key in ['w', 's'] and not autonomous_mode:
        print("Motor stopped")
        motor_hat.set_throttle(0)

def main():
    print("Press 'w' for forward, 's' for backward, 'a' for left, 'd' for right")
    print("Press 'm' to toggle autonomous mode, 't' to stop, 'esc' to quit")
    listen_keyboard(
        on_press=press,
        on_release=release,
        sequential=True
    )

    # 프로그램 종료 시 정리
    motor_hat.set_throttle(0)  # 모터 정지
    servo_controller.set_angle(100)  # 서보 모터 초기 위치로 리셋
    pca.deinit()  # PCA9685 정리
    print("Program stopped and motor stopped.")

if __name__ == "__main__":
    main()
