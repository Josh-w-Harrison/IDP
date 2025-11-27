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
            self.dir_pin.value(direction)
            self.pwm.duty_u16(int(65535 * abs(speed) / 100))


class Actuator:

    def __init__(self, dir_pin, pwm_pin):
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.off()
        self.base_speed = 75
        self.height = 0
        
        self.reset()

    def off(self):
        """Stop the motor."""
        self.pwm.duty_u16(0)

    def speed(self, speed):
        """Set motor direction and speed. Positive = forward, negative = reverse, 0 = stop."""
        if speed == 0:
            self.off()
        else:
            direction = 1 if speed < 0 else 0
            self.dir_pin.value(direction)
            self.pwm.duty_u16(int(65535 * abs(speed) / 100))
            
    def reset(self):
        self.speed(-self.base_speed)
        sleep(5)
        self.off()
            
    def set_height(self, height):
        if height > self.height:
            self.speed(self.base_speed)
        else:
            self.speed(-self.base_speed)
            
        sleep(abs(height - self.height) / (0.07 * self.base_speed))
        
        self.off()
        

if __name__ == "__main__":
    actuator = Actuator(0, 1)
    
    # actuator.speed(50)
    actuator.reset()
    actuator.set_height(10)
