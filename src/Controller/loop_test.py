import sys
sys.path.append('src/Actuators')

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

def read_sensor(pin_num):
    sensor = Pin(pin_num, Pin.IN, Pin.PULL_UP)  # <-- use pull-up instead
    return sensor.value()  # 1=white (on line), 0=black (off line)

# === LINE SENSORS (ADJUST PINS TO YOUR BOARD!) ===
s16 = read_sensor(9)   # leftmost
s17 = read_sensor(10)
s18 = read_sensor(11)
s19 = read_sensor(12)  # rightmost

base_speed = 60
turn_speed = 35

def get_line_binary():
    s16 = read_sensor(9)   # leftmost
    s17 = read_sensor(10)
    s18 = read_sensor(11)
    s19 = read_sensor(12)  # rightmost`
    return (s16 << 3) | (s17 << 2) | (s18 << 1) | s19

def detect_junction(line_binary):
    # FULL JUNCTION (1111 = 15)
    if line_binary == 15:
        return "FULL"

    # LEFT JUNCTION (1110 = 14)
    if line_binary == 14:
        return "LEFT"

    # RIGHT JUNCTION (0111 = 7)
    if line_binary == 7:
        return "RIGHT"

    return None

# List of all logical junctions encountered
logical_junctions = []


def junction_handler(junction_count, block_picked=False, block_is_blue=False):
    print("Detected junction (logical count):", junction_count)
    logical_junctions.append(junction_count)

    # mimic your C logic
    green_box = (junction_count == 2 and block_is_blue and block_picked)
    red_box   = (junction_count == 4 and not block_is_blue and block_picked)

    if junction_count == 1 or green_box or red_box:
        print("Performing right junction turn")
        junction_turn(right=False)

    if green_box or red_box:
        print("Resetting junction counter after drop")
        return 0  # reset
    return junction_count


def junction_turn(right=True):
    print("Turning at junction")
    turn_time = 0.9

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


def line_following(motor_left, motor_right, base_speed=60, soft_turn_speed=35, hard_turn_speed=25):

    junction_count = 0
    last_junc_time = 0
    junction_index = 0

    # IGNORE LIST (unchanged)
    IGNORE_SET = {1, 2, 3, 4, 6, 7,8,9,10,11, 13, 15, 16, 17, 18, 19, 20}

    print("=== Entering continuous line-following loop ===")

    while True:
        s16 = read_sensor(9)   # leftmost
        s17 = read_sensor(10)
        s18 = read_sensor(11)
        s19 = read_sensor(12)  # rightmost

        # --- JUNCTION DETECTION ---
        if (s16 == 1 and s17 == 1 and s18 == 1 and s19 == 1) or \
           (s17 == 1 and s18 == 1 and (s16 == 1 or s19 == 1)):

            now = ticks_ms()
            if now - last_junc_time > 1000:
                last_junc_time = now
                junction_index += 1

                print("JUNCTION SEEN =", junction_index)

                # ignore logic
                if junction_count in IGNORE_SET:
                    print("IGNORING logical junction", junction_count)
                    motor_left.Forward(base_speed)
                    motor_right.Forward(base_speed)
                    sleep(0.2)
                    continue

                junction_count += 1
                junction_count = junction_handler(junction_count)

            # after handling → straight
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)
            sleep(0.2)
            continue

        # --- LINE FOLLOW LOGIC ---
        if s17 == 1 and s18 == 1:
            print("CENTERED → forward")
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)
            continue

        if s17 == 0 and s18 == 1:
            print("DRIFT LEFT → RIGHT correction")
            motor_left.Forward(soft_turn_speed)
            motor_right.Forward(base_speed)
            continue

        if s17 == 1 and s18 == 0:
            print("DRIFT RIGHT → LEFT correction")
            motor_left.Forward(base_speed)
            motor_right.Forward(soft_turn_speed)
            continue

        if s16 == 0:
            print("HARD LEFT DRIFT → HARD RIGHT")
            motor_left.Forward(hard_turn_speed)
            motor_right.Forward(base_speed)
            continue

        if s19 == 0:
            print("HARD RIGHT DRIFT → HARD LEFT")
            motor_left.Forward(base_speed)
            motor_right.Forward(hard_turn_speed)
            continue

        # LOST LINE
        print("LOST LINE → STOP")
        motor_left.off()
        motor_right.off()
        continue


def follow_line_for_distance(motor_left, motor_right, distance_m, speed=60):

    base_speed_m_per_s = 0.2 * (speed / 60)
    target_time = distance_m / base_speed_m_per_s

    start = ticks_ms()

    # run the line-following loop until distance travelled
    while (ticks_ms() - start) < target_time * 1000:
        line_following(motor_left, motor_right, base_speed=60)



def grab():
    print("GRAB closing gripper")
    sleep(1)


def release():
    print("RELEASE opening gripper")
    sleep(1)

def manual_navigation():
    motor_left = Motor(dirPin=4, PWMPin=5)
    motor_right = Motor(dirPin=7, PWMPin=6)

    print("Starting manual navigation sequence...")

    # Step 1: Forward 0.4 m (time-based as before)
    move_forward(motor_left, motor_right, speed=70, distance_m=0.55)

    # Step 3: LINE FOLLOWING instead of blind forward 1.1 m
    print("STEP 3: line-following for ~3.0 m")
    line_following(motor_left, motor_right, base_speed=60, soft_turn_speed=35, hard_turn_speed=25)

    print("Sequence complete.")


if __name__ == "__main__":
    manual_navigation()