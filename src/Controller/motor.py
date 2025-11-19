from machine import Pin, PWM


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



