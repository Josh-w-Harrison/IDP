from machine import Pin, PWM
from utime import sleep, ticks_ms

import sys
sys.path.append('src/Actuators')
sys.path.append('src/Sensors')
sys.path.append('src/Controller')
from Sensors import sensor_pins
from Controller import state_machine


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



def junction_handler(junction_count, block_picked=False, block_is_blue=False):
    print("Detected junction (logical count):", junction_count)

    # mimic your C logic
    green_box = (junction_count == 2 and block_is_blue and block_picked)
    red_box   = (junction_count == 4 and not block_is_blue and block_picked)

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

    left_motor = state_machine.Motor(4, 5)
    right_motor =state_machine.Motor(7, 6)

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

    junction_count = 0       # logical counter for your handler
    last_junc_time = 0

    print("Starting line following...")

    # === STARTUP: go straight for 0.5 m ignoring junctions ===
    startup_distance = 0.5
    startup_speed = 60
    base_speed_m_per_s = 0.2 * (startup_speed / 60)
    startup_time = startup_distance / base_speed_m_per_s

    print("Startup: Going straight for 0.5 m ignoring junctions")

    motor_left.Forward(startup_speed)
    motor_right.Forward(startup_speed)
    sleep(startup_time)

    motor_left.off()
    motor_right.off()

    print("Startup finished → turning LEFT")
    junction_turn(right=False)

    # Junction pattern:
    # ignore 1,2,3,12 and handle all others
    IGNORE_SET = {1, 2, 3, 4, 6, 7,8,9,10,11, 13, 15, 16, 17, 18, 19, 20}
    junction_index = 0   # physical junction index (1,2,3,... as seen)

    while True:
        left_junction_sensor = sensor_pins.read_sensor(8)
        left_line_sensor = sensor_pins.read_sensor(9)
        right_line_sensor = sensor_pins.read_sensor(10)
        right_junction_sensor = sensor_pins.read_sensor(11)

        # Junction detection logic
        if (left_junction_sensor == 1 and left_line_sensor == 1 and right_line_sensor == 1 and right_junction_sensor == 1) or \
           (left_line_sensor == 1 and right_line_sensor == 1 and (left_junction_sensor == 1 or right_junction_sensor == 1)):

            now = ticks_ms()
            if now - last_junc_time > 1000:  # debounce
                last_junc_time = now
                junction_index += 1

                print("JUNCTION SEEN =", junction_index)

                # ================================
                # IGNORE BEHAVIOUR
                # ================================
                if junction_index in IGNORE_SET:
                    print("IGNORING junction", junction_index)
                    motor_left.Forward(base_speed)
                    motor_right.Forward(base_speed)
                    sleep(0.2)
                    continue

                # ================================
                # NORMAL HANDLING
                # ================================
                junction_count += 1
                junction_count = junction_handler(junction_count)

            # default straight movement after handling
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)
            sleep(0.2)
            continue

        # === CENTERED — middle pair on white ===
        if left_line_sensor == 1 and right_line_sensor == 1:
            print("CENTERED → forward")
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)
            return

        # === SLIGHT DRIFT LEFT — mid-left sees dark ===
        if left_line_sensor == 0 and right_line_sensor == 1:
            print("DRIFT LEFT → turn RIGHT")
            motor_left.Forward(soft_turn_speed)
            motor_right.Forward(base_speed)
            return

        # === SLIGHT DRIFT RIGHT — mid-right sees dark ===
        if left_line_sensor == 1 and right_line_sensor == 0:
            print("DRIFT RIGHT → turn LEFT")
            motor_left.Forward(base_speed)
            motor_right.Forward(soft_turn_speed)
            return

        # === HARD DRIFT LEFT — leftmost sees line ===
        if left_junction_sensor == 0:
            print("HARD LEFT DRIFT → HARD RIGHT")
            motor_left.Forward(hard_turn_speed)
            motor_right.Forward(base_speed)
            return

        # === HARD DRIFT RIGHT — rightmost sees line ===
        if right_junction_sensor == 0:
            print("HARD RIGHT DRIFT → HARD LEFT")
            motor_left.Forward(base_speed)
            motor_right.Forward(hard_turn_speed)
            return

        # === LOST LINE ===
        print("LOST LINE → STOP")
        motor_left.off()
        motor_right.off()


def follow_line_for_distance(motor_left, motor_right, distance_m, speed=60):
    """
    Line follow for a specific distance in meters.
    Uses the 4-sensor line following logic.
    Ignores junctions.
    """

    # Convert distance to time using your linear model
    base_speed_m_per_s = 0.2 * (speed / 60)
    target_time = distance_m / base_speed_m_per_s

    start = ticks_ms()

    while (ticks_ms() - start) < target_time * 1000:
        line_following(motor_left, motor_right, base_speed=60, soft_turn_speed=35, hard_turn_speed=25)
        sleep(0.01)

    # stop at end
    motor_left.off()
    motor_right.off()



if __name__ == "__main__":
    motor_left = state_machine.Motor(dirPin=4, PWMPin=5)
    motor_right = state_machine.Motor(dirPin=7, PWMPin=6)

    # STARTUP: go straight 0.5m
    startup_distance = 0.5
    startup_speed = 60
    base_speed_m_per_s = 0.2 * (startup_speed / 60)
    startup_time = startup_distance / base_speed_m_per_s

    motor_left.Forward(startup_speed)
    motor_right.Forward(startup_speed)
    sleep(startup_time)

    motor_left.off()
    motor_right.off()

    state_machine.turn_left(motor_left, motor_right)   # MANUAL LEFT TURN

    follow_line_for_distance(motor_left, motor_right, 2.2, speed=60)