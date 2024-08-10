# from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import time

class PWMThrottleHat:
    def __init__(self, pwm, channel):
        """
        PWMThrottleHat Class init
        :param pwm: PCA9685 address
        :param channel: control channel
        """
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60 # set freq.

    
    def set_throttle(self, throttle):
        """
        Throttle Motor Control
        :param throttle: -1 ~ 1 value range
        """

        def set_channel_values(ch5, ch4, ch3):
            self.pwm.channels[self.channel + 5].duty_cycle = ch5
            self.pwm.channels[self.channel + 4].duty_cycle = ch4
            self.pwm.channels[self.channel + 3].duty_cycle = ch3
        
        pulse = int(0xFFFF * abs(throttle)) # 16 bits duty cycle

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


def main(args=None):
    driver = SteerThrottle()
    driver.set_throttle(0)
    driver.set_steer(90)
    time.sleep(0.05) # depend on Hz


if __name__ == '__main__':
    main()
