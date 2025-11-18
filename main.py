import sys
sys.path.append('src')

from machine import Pin, PWM
from utime import sleep, ticks_ms
from Controller.navigation import maze_map, find_shortest_path, relative_turn

# ------------------ GLOBAL STATE ------------------

junction_detected = False
stopped = True
current_path_index = 0
robot_state = "line_following"  # "line_following", "navigating", "completed"


def junction_interrupt(pin):
    global junction_detected
    print(f"INTERRUPT FIRED on pin {pin}! Sensor value: {pin.value()}")
    junction_detected = True

def button_interrupt(pin):
    global stopped
    print(f"INTERRUPT FIRED on pin {pin}! Sensor value: {pin.value()}")
    stopped = not stopped


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


def move_forward(left, right, speed=35, dist_m=0.1):
    base_speed = 0.2 * (speed / 60)
    t = dist_m / base_speed
    left.Forward(speed); right.Forward(speed); sleep(t)
    left.off(); right.off()


def turn(left, right, rel_dir, speed=75):
    # rel_dir: +1 = right 90°, -1 = left 90°, 2 = U-turn
    # Turn for minimum time then until junction sensors (pins 8 or 11) detect a line
    
    print(f"Starting turn with rel_dir={rel_dir}, speed={speed}")
    start_time = ticks_ms()
    min_turn_time = 500  # 1.5 seconds minimum turn time
    
    if rel_dir == +1:
        # Turn right until junction sensor detects line
        print("Turning right...")
        left.Forward(speed); right.Reverse(speed)
        # Wait minimum time first
        while ticks_ms() - start_time < min_turn_time:
            pass  # Keep turning for minimum time
        # Then check for line detection
        while read_sensor(9) == 0:  # Continue turning while no line detected
            pass  # Keep turning
        left.off(); right.off()
        print("Right turn completed")
        
    elif rel_dir == -1:
        # Turn left until junction sensor detects line
        print("Turning left...")
        left.Reverse(speed); right.Forward(speed)
        # Wait minimum time first
        while ticks_ms() - start_time < min_turn_time:
            pass  # Keep turning for minimum time
        # Then check for line detection
        while read_sensor(10) == 0:  # Continue turning while no line detected
            pass  # Keep turning
        left.off(); right.off()
        print("Left turn completed")
        
    elif rel_dir == 2:
        # U-turn: turn until line detected twice (180 degrees)
        print("Starting U-turn...")
        left.Forward(speed); right.Reverse(speed)
        # Wait minimum time first (double for U-turn)
        while ticks_ms() - start_time < min_turn_time * 2:
            pass  # Keep turning for minimum time
        # First, turn past the current line
        while read_sensor(9) == 0 or read_sensor(10) == 0:  # Turn while still on line
            pass  # Keep turning past current line
        print("Passed current line, continuing turn...")
        # Then continue turning until we find the line again
        while read_sensor(8) == 1 and read_sensor(11) == 1:  # Continue turning while no line detected
            pass  # Keep turning until line found again
        left.off(); right.off()
        print("U-turn completed")


# ------------------ LINE SENSING ------------------

def read_sensor(pin_num):
    return Pin(pin_num, Pin.IN, Pin.PULL_UP).value()


def line_follow(left, right):
    base = 80
    diff = (read_sensor(9) - read_sensor(10)) * 15
    #left.Forward(base - diff); right.Forward(base + diff)
    left.Reverse(base+diff); right.Reverse(base-diff)


def navigate_path(path):
    global junction_detected, current_path_index, robot_state
    
    left, right = Motor(4, 5), Motor(7, 6)
    direction = 0  # starting orientation
    
    # Set up junction detection interrupts on pins 8 and 11
    junction_pin1 = Pin(8, Pin.IN, Pin.PULL_UP)
    junction_pin2 = Pin(11, Pin.IN, Pin.PULL_UP)
    junction_pin1.irq(trigger=Pin.IRQ_RISING, handler=junction_interrupt)
    junction_pin2.irq(trigger=Pin.IRQ_RISING, handler=junction_interrupt)

    robot_state = "line_following"
    current_path_index = 0

    print(robot_state)

    try:
        while current_path_index < len(path) - 1:

            print( f"Current robot state: {robot_state}, at path index: {current_path_index}" )

            if stopped:
                break

            if robot_state == "line_following":
                # Continue line following until junction detected
                print(f"Line following mode, looking for junction to go from index {current_path_index} to {current_path_index + 1}")
                print(f"Current sensors - Pin 8: {read_sensor(8)}, Pin 11: {read_sensor(11)}")

                while not junction_detected:
                    line_follow(left, right)

                if stopped:
                    break
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
                            if stopped:
                                break
                        else:
                            print("No turn needed - going straight")
                        
                        # Update direction
                        direction = new_dir
                        
                        # Move forward slightly to clear junction
                        # move_forward(left, right, dist_m=0.1)
                        
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
    return


# ------------------ MAIN ------------------

if __name__ == "__main__":
    # Example: Navigate from BoxInside to Red
    start_node = "BoxInside"
    end_node = "UpperRackA3"
    
    # # Find the shortest path using the navigation module
    path = find_shortest_path(start_node, end_node)
    
    # print(path)

    # path = ["BoxInside", "BoxEntrance", "BoxJunction", "GreenJunction", "BlueJunction", "LowerRackA6", "LowerRackA5", "LowerRackA4", "LowerRackA3", "LowerRackA2", "LowerRackA1", "LeftCross", "BackLeftTurn", "LowerRamp", "BackRightTurn", "RightCross", "LowerRampB6", "LowerRackB5", "LowerRackB4", "LowerRackB3", "LowerRackB2", "LowerRackB1", "RedJunction", "YellowJunction", "BoxJunction", "BoxEntrance", "BoxInside"]

    # leftMotor = Motor(4, 5)
    # rightMotor = Motor(7, 6)
    # try:
    #     while True:
    #         line_follow(leftMotor, rightMotor)
    # except KeyboardInterrupt:
    #     leftMotor.off()
    #     rightMotor.off()

    button_pin = Pin(12, Pin.IN, Pin.PULL_DOWN)
    button_pin.irq(trigger=Pin.IRQ_RISING, handler=button_interrupt)

    print(stopped)

    while True:
        if not stopped:
            navigate_path(path)
