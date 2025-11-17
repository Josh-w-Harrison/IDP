from machine import Pin, PWM
from utime import sleep, ticks_ms

# ------------------ GLOBAL STATE ------------------

junction_detected = False
current_path_index = 0
robot_state = "line_following"  # "line_following", "navigating", "completed"


def junction_interrupt(pin):
    global junction_detected
    print(f"INTERRUPT FIRED on pin {pin}! Sensor value: {pin.value()}")
    junction_detected = True


# ------------------ MOTOR CONTROL ------------------

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.off()

    def off(self): self.pwm.duty_u16(0)

    def Forward(self, speed=60): self._move(0, speed)

    def Reverse(self, speed=60): self._move(1, speed)

    def _move(self, direction, speed):
        self.mDir.value(direction)
        self.pwm.duty_u16(int(65535 * speed / 100))


def move_forward(left, right, speed=60, dist_m=0.3):
    base_speed = 0.2 * (speed / 60)
    t = dist_m / base_speed
    left.Forward(speed);
    right.Forward(speed);
    sleep(t)
    left.off();
    right.off()


def turn(left, right, rel_dir, speed=60):
    # rel_dir: +1 = right 90°, -1 = left 90°, 2 = U-turn
    # Turn for minimum time then until junction sensors (pins 8 or 11) detect a line

    print(f"Starting turn with rel_dir={rel_dir}, speed={speed}")
    start_time = ticks_ms()
    min_turn_time = 1500  # 1.5 seconds minimum turn time

    if rel_dir == +1:
        # Turn right until junction sensor detects line
        print("Turning right...")
        left.Forward(speed);
        right.Reverse(speed)
        # Wait minimum time first
        while ticks_ms() - start_time < min_turn_time:
            pass  # Keep turning for minimum time
        # Then check for line detection
        while read_sensor(8) == 1 and read_sensor(11) == 1:  # Continue turning while no line detected
            pass  # Keep turning
        left.off();
        right.off()
        print("Right turn completed")

    elif rel_dir == -1:
        # Turn left until junction sensor detects line
        print("Turning left...")
        left.Reverse(speed);
        right.Forward(speed)
        # Wait minimum time first
        while ticks_ms() - start_time < min_turn_time:
            pass  # Keep turning for minimum time
        # Then check for line detection
        while read_sensor(8) == 1 and read_sensor(11) == 1:  # Continue turning while no line detected
            pass  # Keep turning
        left.off();
        right.off()
        print("Left turn completed")

    elif rel_dir == 2:
        # U-turn: turn until line detected twice (180 degrees)
        print("Starting U-turn...")
        left.Forward(speed);
        right.Reverse(speed)
        # Wait minimum time first (double for U-turn)
        while ticks_ms() - start_time < min_turn_time * 2:
            pass  # Keep turning for minimum time
        # First, turn past the current line
        # Then continue turning until we find the line again
        while read_sensor(9) == 1 and read_sensor(10) == 1:  # Continue turning while no line detected
            pass  # Keep turning until line found again
        left.off();
        right.off()
        print("U-turn completed")

def reverse_turn(left, right, rel_dir, speed=60):
    print(f"Starting turn with rel_dir={rel_dir}, speed={speed}")
    start_time = ticks_ms()
    min_turn_time = 3000  # 1.5 seconds minimum turn time

    # you want robot to end up facing foward in right direction after reversing
    # rel_dir: +1 = right 90°, -1 = left 90°,
    if rel_dir == +1:
        print("Turning right...")
        left.Reverse(speed);
        right.Forward(speed)
        while ticks_ms() - start_time < min_turn_time*2:
            pass  # Keep turning for minimum time
        #then check for line detection
        while read_sensor(8) == 1 and read_sensor(11) == 1:  # Continue turning while no line detected
            pass  # Keep turning
        left.off();
        right.off()
        print("Right turn completed")
    elif rel_dir == -1:
        print("Turning left...")
        left.Forward(speed);
        right.Reverse(speed)
        while ticks_ms() - start_time < min_turn_time*2:
            pass  # Keep turning for minimum time
        #then check for line detection
        while read_sensor(8) == 1 and read_sensor(11) == 1:  # Continue turning while no line detected
            pass  # Keep turning
        left.off();
        right.off()
        print("Left turn completed")

    elif rel_dir == 2:
        # U-turn: turn until line detected twice (180 degrees)
        print("Starting U-turn...")
        left.Forward(speed);
        right.Reverse(speed)
        # Wait minimum time first (double for U-turn)
        while ticks_ms() - start_time < min_turn_time * 2:
            pass  # Keep turning for minimum time
        # First, turn past the current line
        while read_sensor(8) == 0 or read_sensor(11) == 0:  # Turn while still on line
            pass  # Keep turning past current line
        print("Passed current line, continuing turn...")
        # Then continue turning until we find the line again
        while read_sensor(8) == 1 and read_sensor(11) == 1:  # Continue turning while no line detected
            pass  # Keep turning until line found again
        left.off();
        right.off()
        print("U-turn completed")
# ------------------ LINE SENSING ------------------

def read_sensor(pin_num):
    return Pin(pin_num, Pin.IN, Pin.PULL_UP).value()


def line_follow(left, right):
    base, diff = 80, 20

    LT, RT = [read_sensor(i) for i in (9, 10)]

    if LT == 1 and RT == 1:
        left.Forward(base);
        right.Forward(base)

    if LT == 1 and RT == 0:
        left.Forward(base - diff);
        right.Forward(base)

    if LT == 0 and RT == 1:
        left.Forward(base);
        right.Forward(base - diff)

    if LT == 0 and RT == 0:
        left.off();
        right.off()

def reverse_line_follow(left, right):
    base, diff = 80, 20

    LT, RT = [read_sensor(i) for i in (9, 10)]

    if LT == 1 and RT == 1:
        left.Reverse(base);
        right.Reverse(base)

    if LT == 1 and RT == 0:
        left.Reverse(base);
        right.Reverse(base - diff)
        print("turning right while reversing")

    if LT == 0 and RT == 1:
        left.Reverse(base - diff);
        right.Reverse(base)
        print("turning left while reversing")

    if LT == 0 and RT == 0:
        left.off();
        right.off()


# ------------------ GRAPH NAVIGATION ------------------

# Map[node][neighbor] = direction (0=N, 1=E, 2=S, 3=W)
maze_map = {
    0: {1: 0},
    1: {2: 3},
    2: {3: 2, 4: 3},
    4: {5: 2, 6: 0},
    6: {7: 0},
    7: {8: 0},
    8: {9: 0},
    9: {10: 0},
    10: {11: 0},
    11: {12: 0},
    12: {13: 0},
    13: {14: 1},
    14: {15: 1},
    15: {16: 2},
    16: {17: 2},
    17: {18: 2},
    18: {19: 2},
    19: {20: 2},
    20: {21: 2},
    21: {22: 2},
    22: {23: 2},
    23: {24: 3},
    24: {1: 3},
}


def relative_turn(current_dir, new_dir):
    diff = (new_dir - current_dir) % 4
    if diff == 1:  return +1  # turn right
    if diff == 3:  return -1  # turn left
    if diff == 2:  return 2  # U-turn
    return 0  # straight


def navigate_path(path):
    global junction_detected, current_path_index, robot_state

    left, right = Motor(4, 5), Motor(7, 6)
    direction = 3  # starting orientation

    # Set up junction detection interrupts on pins 8 and 11
    junction_pin1 = Pin(8, Pin.IN, Pin.PULL_UP)
    junction_pin2 = Pin(11, Pin.IN, Pin.PULL_UP)
    junction_pin1.irq(trigger=Pin.IRQ_RISING, handler=junction_interrupt)
    junction_pin2.irq(trigger=Pin.IRQ_RISING, handler=junction_interrupt)

    robot_state = "line_following"
    current_path_index = 0

    try:
        while current_path_index < len(path) - 1:
            if robot_state == "line_following":
                # Continue line following until junction detected
                print(
                    f"Line following mode, looking for junction to go from index {current_path_index} to {current_path_index + 1}")
                print(f"Current sensors - Pin 8: {read_sensor(8)}, Pin 11: {read_sensor(11)}")

                while not junction_detected:
                    line_follow(left, right)
                    # Add periodic sensor checking for debugging
                    # if ticks_ms() % 1000 < 50:  # Print every ~1 second
                    #     print(f"Still line following... Pin 8: {read_sensor(8)}, Pin 11: {read_sensor(11)}")

                # Check if junction was detected
                if junction_detected:
                    # Update to the current node first
                    current_path_index += 1
                    current_node = path[current_path_index]

                    print(f"Junction detected! Now at node {current_node}")
                    left.off()
                    right.off()
                    sleep(0.1)  # Brief stop at junction

                    # Check if we need to turn from this junction
                    if current_path_index < len(path) - 1:
                        next_node = path[current_path_index + 1]
                        new_dir = maze_map[current_node][next_node]
                        rel = relative_turn(direction, new_dir)

                        print(f"At node {current_node}, going to {next_node}, rel_turn={rel}")
                        print(f"Current direction: {direction}, New direction: {new_dir}")
                        print(f"Available connections from node {current_node}: {maze_map.get(current_node, 'None')}")

                        # Execute turn if needed
                        if rel != 0:
                            print(f"Executing turn: {rel}")
                            turn(left, right, rel)
                        else:
                            print("No turn needed - going straight")

                        # Update direction
                        direction = new_dir

                        # Move forward slightly to clear junction
                        move_forward(left, right, dist_m=0.1)

                        print(f"Completed navigation from node {current_node}, continuing line following")

                    # Reset junction detection
                    junction_detected = False
                    robot_state = "line_following"

        print("Path navigation completed!")
        robot_state = "completed"

    except KeyboardInterrupt:
        print("Navigation interrupted by user")
        left.off()
        right.off()

    left.off()
    right.off()

def test_reverse():
    left, right = Motor(4, 5), Motor(7, 6)
    turn(left, right, 2)  # U-turn to face backward
#follow line in reverse until junction detected then make right turn in reverse to forward direction


    left.off()
    right.off()


# ------------------ MAIN ------------------

if __name__ == "__main__":
    #path = [1, 2, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
    #navigate_path(path)
    test_reverse()


