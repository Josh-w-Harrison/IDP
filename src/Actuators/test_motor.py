from machine import Pin, PWM
from utime import sleep

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
        
    def off(self):
        self.pwm.duty_u16(0)
        
    def Forward(self, speed=100):
        self.mDir.value(0)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

    def Reverse(self, speed=30):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))
        
    # TURNING RIGHT AND LEFT 
    def turn_left(self, left_motor, right_motor, speed=50):
        print("Turning left")
        left_motor.Reverse(speed)
        right_motor.Forward(speed)
        
    def turn_right(self, left_motor, right_motor, speed=50):
        print("Turning right")
        left_motor.Forward(speed)
        right_motor.Reverse(speed)

def line_sensor_16():
    "Simple poll of input"
    input_pin = 16  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)

def line_sensor_17():
    "Simple poll of input"
    input_pin = 17  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)


def line_sensor_18():
    "Simple poll of input"
    input_pin = 18  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)

def line_sensor_19():
    "Simple poll of input"
    input_pin = 19  # Pin 19 = GP19 (labelled 23 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)

        
def line_following():
    motor_left = Motor(dirPin=4, PWMPin=5)   # Left motor
    motor_right = Motor(dirPin=7, PWMPin=6)  # Right motor
    base_speed = 70
    turn_speed = 40

    while True:
        # Get readings from all four sensors
        val16 = line_sensor_16()
        val17 = line_sensor_17()
        val18 = line_sensor_18()
        val19 = line_sensor_19()
        print(f"Sensors: 16={val16}, 17={val17}, 18={val18}, 19={val19}")

        #Logic
        # (1 = on line / white, 0 = off line / black)
        if val17 == 1 and val18 == 1:
            # centered on the line
            print("CENTERED → Forward")
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)

        elif val16 == 0:
            # line is more to the left → turn right
            print("LEFT EDGE → Turn right")
            motor_left.Forward(turn_speed)
            motor_right.Forward(base_speed)

        elif val19 == 0:
            # line is more to the right → turn left
            print("RIGHT EDGE → Turn left")
            motor_left.Forward(base_speed)
            motor_right.Forward(turn_speed)

        else:
            # lost the line completely
            print("LOST LINE → Stop")
            motor_left.off()
            motor_right.off()

        sleep(0.2)


if __name__ == "__main__":
    line_following()