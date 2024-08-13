from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import time
import os

class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60

    def set_throttle(self, throttle):
        def set_channel_values(ch5, ch4, ch3):
            self.pwm.channels[self.channel + 5].duty_cycle = ch5
            self.pwm.channels[self.channel + 4].duty_cycle = ch4
            self.pwm.channels[self.channel + 3].duty_cycle = ch3

        pulse = int(0xFFFF * abs(throttle))

        if throttle > 0:
            set_channel_values(pulse, 0, 0xFFFF)
        elif throttle < 0:
            set_channel_values(pulse, 0xFFFF, 0)
        else:
            set_channel_values(0, 0, 0)


class SteerThrottle:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60

        self.throttle_driver = PWMThrottleHat(self.pca, channel=0)
        self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)

    def set_throttle(self, throttle):
        self.throttle_driver.set_throttle(throttle)

    def set_steer(self, steer):
        self.kit.servo[0].angle = steer


def start_pipe_server(pipe_name='/tmp/steer_throttle_pipe'):
    driver = SteerThrottle()

    # Ensure the pipe doesn't exist before creating it
    if not os.path.exists(pipe_name):
        os.mkfifo(pipe_name)

    with open(pipe_name, 'r') as pipe:
        print(f'Pipe server listening on {pipe_name}')
        while True:
            data = pipe.readline().strip()
            if not data:
                continue
            try:
                steer, throttle = map(float, data.split(','))
                driver.set_steer(steer)
                driver.set_throttle(throttle)
                
                current_time_sec = time.time()
                # current_time_ns = time.time_ns() % 1_000_000_000  # 나노초 단위 추출
                print(f'[{current_time_sec:.9f}] Pipe server set steer: {steer} throttle: {throttle}')
            except Exception as e:
                print(f'Error: {e}')
            time.sleep(0.033) # 30 Hz

if __name__ == '__main__':
    start_pipe_server()
