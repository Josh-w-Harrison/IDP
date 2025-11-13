from machine import Pin, PWM
from utime import sleep, ticks_ms

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed=60):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed=60):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))


def read_sensor(pin_num):
    sensor = Pin(pin_num, Pin.IN, Pin.PULL_UP)
    return sensor.value()   # 1 = white, 0 = black


def junction_handler(junction_count, block_picked=False, block_is_blue=False):
    print("Detected junction:", junction_count)

    # mimic your C logic
    green_box = (junction_count == 2 and block_is_blue and block_picked)
    red_box = (junction_count == 4 and not block_is_blue and block_picked)

    if junction_count == 1 or green_box or red_box:
        print("Performing right junction turn")
        junction_turn(right=True)

    if green_box or red_box:
        print("Resetting junction counter after drop")
        return 0  # reset
    return junction_count


def junction_turn(right=True):
    print("Turning at junction")
    turn_time = 0.9
    dist_time = 1.2

    left_motor = Motor(4, 5)
    right_motor = Motor(7, 6)

    # go straight before turning
    left_motor.Forward(60)
    right_motor.Forward(60)
    sleep(0.9)

    if right:
        left_motor.Forward(60)
        right_motor.Reverse(60)
    else:
        left_motor.Reverse(60)
        right_motor.Forward(60)
    sleep(turn_time)

    left_motor.off()
    right_motor.off()


def line_following():
    motor_left = Motor(4, 5)
    motor_right = Motor(7, 6)

    base_speed = 60
    turn_speed = 35

    junction_count = 0
    last_junc_time = 0

    print("Starting line following...")

    while True:
        s16 = read_sensor(16)
        s17 = read_sensor(17)
        s18 = read_sensor(18)
        s19 = read_sensor(19)

        # Junction detection (all sensors on line or 3 sensors on line)
        if (s16 == 1 and s17 == 1 and s18 == 1 and s19 == 1) or \
           (s17 == 1 and s18 == 1 and (s16 == 1 or s19 == 1)):
            now = ticks_ms()
            if now - last_junc_time > 1000:  # debounce junction detection
                junction_count += 1
                last_junc_time = now
                junction_count = junction_handler(junction_count)
            # continue straight briefly
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)
            sleep(0.2)
            continue

        # Line following logic
        if s17 == 1 and s18 == 1:
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)

        elif s17 == 0 and s18 == 1:
            motor_left.Forward(turn_speed)
            motor_right.Forward(base_speed)

        elif s17 == 1 and s18 == 0:
            motor_left.Forward(base_speed)
            motor_right.Forward(turn_speed)

        else:
            motor_left.off()
            motor_right.off()

        sleep(0.05)


if __name__ == "__main__":
    line_following()
