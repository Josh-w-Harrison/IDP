import sys
sys.path.append('src/Actuators')

from machine import Pin, PWM
from utime import sleep


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

    def _move(self, direction, speed):
        self.mDir.value(direction)
        self.pwm.duty_u16(int(65535 * speed / 100))

def move_forward(motor_left, motor_right, speed, distance_m):
    # Distance → approximate duration (calibration needed!)
    # Assuming 0.2 m/s base speed at 60% PWM → scale linearly
    base_speed_m_per_s = 0.2 * (speed / 60)
    time_s = distance_m / base_speed_m_per_s
    print(f"FORWARD {distance_m:.2f} m  (≈ {time_s:.2f} s)")

    motor_left.Forward(speed)
    motor_right.Forward(speed)
    sleep(time_s)
    motor_left.off()
    motor_right.off()

def move_reverse(motor_left, motor_right, speed, distance_m):
    base_speed_m_per_s = 0.2 * (speed / 60)
    time_s = distance_m / base_speed_m_per_s
    print(f"REVERSE {distance_m:.2f} m ({time_s:.2f} s)")
    motor_left.Reverse(speed); motor_right.Reverse(speed)
    sleep(time_s)
    motor_left.off(); motor_right.off()


def turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0):
    print(f"TURN LEFT for {turn_time:.2f} s")
    motor_left.Reverse(turning_speed)
    motor_right.Forward(turning_speed)
    sleep(turn_time)
    motor_left.off()
    motor_right.off()


def turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0):
    print(f"TURN RIGHT for {turn_time:.2f} s")
    motor_left.Forward(turning_speed)
    motor_right.Reverse(turning_speed)
    sleep(turn_time)
    motor_left.off()
    motor_right.off()


# Return value of each sensor once per call (no infinite loop)
def read_sensor(pin_num):
    sensor = Pin(pin_num, Pin.IN, Pin.PULL_UP)  # <-- use pull-up instead
    return sensor.value()  # 1=white (on line), 0=black (off line)


def line_following():
    motor_left = Motor(dirPin=4, PWMPin=5)
    motor_right = Motor(dirPin=7, PWMPin=6)

    base_speed = 60
    turn_speed = 35

    print("Starting line following...")

    while True:
        # Read all sensors
        s16 = read_sensor(16)
        s17 = read_sensor(17)
        s18 = read_sensor(18)
        s19 = read_sensor(19)

        print(f"Sensors: 16={s16}, 17={s17}, 18={s18}, 19={s19}")

        # Logic: 1 = white (on line), 0 = black (off line)
        if s17 == 1 and s18 == 1:
            # On the line — go forward
            print("CENTERED Forward")
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)

        elif s17 == 0 and s18 == 1:
            # Line drifting left → turn right
            print("LEFT EDGE Turn right")
            motor_left.Forward(turn_speed)
            motor_right.Forward(base_speed)

        elif s16 == 1 or s19 == 0:
            return 0

        else:
            # Lost line — stop
            print("LOST LINE Stop")
            motor_left.off()
            motor_right.off()

        sleep(0.1)



if __name__ == "__main__":
    map = {
        0: {(1, 0)},
        1: {(2, 3)},
        2: [(3, 2), (4, 3)],
        4: [(5, 2), (6, 0)],
    }

    motor_left = Motor(dirPin=4, PWMPin=5)
    motor_right = Motor(dirPin=7, PWMPin=6)

    position = 0
    direction = 0

    new_direction = map[0][1]
    line_following()

    new_direction = map[1][2]
    if new_direction == 1:
        turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    if new_direction == 3:
        turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    line_following()