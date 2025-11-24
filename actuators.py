from machine import Pin, PWM
from utime import sleep


class Motor:
    """Controls a single DC motor with direction and PWM speed control."""

    def __init__(self, dir_pin, pwm_pin):
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.off()

    def off(self):
        """Stop the motor."""
        self.pwm.duty_u16(0)

    def speed(self, speed):
        """Set motor direction and speed. Positive = forward, negative = reverse, 0 = stop."""
        if speed == 0:
            self.off()
        else:
            direction = 1 if speed < 0 else 0
            #print(speed, direction)
            self.dir_pin.value(direction)
            self.pwm.duty_u16(int(65535 * abs(speed) / 100))


class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        self.mDir.value(dir)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

