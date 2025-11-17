from machine import Pin, PWM
from utime import sleep

class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        self.mDir.value(dir)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=50):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))