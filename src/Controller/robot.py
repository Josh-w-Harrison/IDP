from machine import Pin
from utime import sleep
from Controller.motor import Motor
from Controller.line_sensor import LineSensor
from Controller.navigation import maze_map, relative_turn


class Robot:
    """Main robot class encapsulating all navigation and control logic."""

    # Robot state constants
    STATE_LINE_FOLLOWING = "line_following"
    STATE_COMPLETED = "completed"

    def __init__(self):
        """Initialize robot with motor and sensor pins."""

        # Motors
        self.left_motor = Motor(4, 5)
        self.right_motor = Motor(7, 6)

        # Sensor pins
        self.line_left_pin = 10
        self.line_right_pin = 9
        self.junction_left_pin = 8
        self.junction_right_pin = 11
        self.button_pin = 12
        self.yellow_light_pin = 28

        # Navigation state
        self.stopped = True
        self.junction_detected = False
        self.robot_state = self.STATE_LINE_FOLLOWING
        self.direction = 0  # Current orientation
        self.node = "BoxInside"
        self.active = False

        # Setup flashing yellow light
        self.yellow_light = Pin(self.yellow_light_pin, Pin.OUT)
        self.yellow_light.value(1)

        # Setup junction detection interrupts
        self.junction_pin1 = Pin(self.junction_left_pin, Pin.IN, Pin.PULL_UP)
        self.junction_pin2 = Pin(self.junction_right_pin, Pin.IN, Pin.PULL_UP)
        self.junction_pin1.irq(trigger=Pin.IRQ_RISING, handler=self._junction_interrupt)
        self.junction_pin2.irq(trigger=Pin.IRQ_RISING, handler=self._junction_interrupt)

        # Setup button interrupt
        self.button = Pin(self.button_pin, Pin.IN, Pin.PULL_DOWN)
        self.button.irq(trigger=Pin.IRQ_RISING, handler=self._button_interrupt)

    def _junction_interrupt(self, pin):
        """Interrupt handler for junction detection."""
        print(f"INTERRUPT: Junction detected on pin {pin}! Value: {pin.value()}")
        self.junction_detected = True

    def _button_interrupt(self, pin):
        """Interrupt handler for button press (stop/start toggle)."""
        print(f"INTERRUPT: Button pressed on pin {pin}! Value: {pin.value()}")
        self.stopped = not self.stopped
        print(f"Robot {'STOPPED' if self.stopped else 'STARTED'}")

    def update_node(self, new_node):
        """Update the current node of the robot."""
        self.node = new_node

        if new_node == "BoxInside":
            self.active = False
            self.yellow_light(1)
        else:
            self.active = True
            self.yellow_light(0)


    def move(self, base_speed, diff_speed=0):
        """Move the robot with differential speeds."""
        self.left_motor.speed(base_speed + diff_speed)
        self.right_motor.speed(base_speed - diff_speed)

    def line_follow(self, base_speed=80, correction_factor=15):
        """Follow a line using differential steering."""
        left_sensor = LineSensor.read(self.line_left_pin)
        right_sensor = LineSensor.read(self.line_right_pin)
        diff = (right_sensor - left_sensor) * correction_factor

        # Reverse direction for your setup
        self.move(base_speed, diff)

    def turn(self, rel_dir, speed=75):
        """
        Execute a turn based on relative direction.
        """""

        match rel_dir:
            case 1:  # Turn right
                self.move(0 , speed)
                while LineSensor.read(self.line_left_pin) == 0:
                    pass
            case -1:  # Turn left
                self.move(0 , -speed)
                while LineSensor.read(self.line_right_pin) == 0:
                    pass
            case 2:  # U-turn
                self.move(0 , speed)
                while LineSensor.read(self.line_right_pin) == 0:
                    pass

        self.move(0)


    def navigate_path(self, path):
        """
        Navigate along a predefined path using line following and junction detection.

        Args:
            path: List of node names representing the path to follow
        """
        # Reset navigation state
        current_path_index = 0
        self.junction_detected = False
        self.robot_state = self.STATE_LINE_FOLLOWING

        print(f"Starting navigation: {path[0]} -> {path[-1]}")

        try:
            while current_path_index < len(path) - 1:

                print(f"State: {self.robot_state}, Path index: {current_path_index}/{len(path)-1}")

                # Check if stopped
                if self.stopped:
                    self.move(0)
                    return

                if self.robot_state == self.STATE_LINE_FOLLOWING:
                    self._follow_line_to_junction(path, current_path_index)

                    if self.stopped:
                        self.move(0)
                        return

                    if self.junction_detected:
                        current_path_index = self._handle_junction(path, current_path_index)

            print("Path navigation completed!")
            self.robot_state = self.STATE_COMPLETED

        except KeyboardInterrupt:
            print("Navigation interrupted by user")
            self.move(0)

        self.move(0)

    def _follow_line_to_junction(self, path, current_path_index):
        """Continue line following until a junction is detected."""
        print(f"Line following mode: {path[current_path_index]} -> {path[current_path_index + 1]}")
        print(f"Sensors - Left junction: {LineSensor.read(self.junction_left_pin)}, "
              f"Right junction: {LineSensor.read(self.junction_right_pin)}")

        while not self.junction_detected and not self.stopped:
            self.line_follow()

    def _handle_junction(self, path, current_path_index):
        """Handle junction detection and execute appropriate turn."""
        # Move to next node
        current_path_index += 1
        current_node = path[current_path_index]

        print(f"Junction detected! Now at node: {current_node}")

        # Check if there's a next node to navigate to
        if current_path_index < len(path) - 1:
            next_node = path[current_path_index + 1]
            new_dir = maze_map[current_node][next_node]
            rel = relative_turn(self.direction, new_dir)

            print(f"Navigation: {current_node} -> {next_node}")
            print(f"Direction change: {self.direction} -> {new_dir} (rel_turn={rel})")

            # Execute turn if needed
            if rel != 0:
                print(f"Executing turn: {rel}")
                self.turn(rel)
                if self.stopped:
                    return current_path_index
            else:
                print("No turn needed - going straight")

            # Update direction
            self.direction = new_dir

            print(f"Completed turn at {current_node}, resuming line following")

        # Reset junction detection
        self.junction_detected = False
        self.robot_state = self.STATE_LINE_FOLLOWING

        return current_path_index

