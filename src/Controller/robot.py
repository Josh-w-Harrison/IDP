import sys
sys.path.append('src')

from machine import Pin, soft_reset, reset
from utime import sleep, ticks_ms
from Controller.motor import Motor
from Controller.line_sensor import LineSensor
from Controller.navigation import maze_map, relative_turn, find_shortest_path


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
        self.line_left_pin = 9
        self.line_right_pin = 10
        self.junction_left_pin = 8
        self.junction_right_pin = 11
        self.button_pin = 12
        self.yellow_light_pin = 28

        # Navigation state
        self.stopped = True
        self.is_reset = True
        self.junction_detected = False
        self.robot_state = self.STATE_LINE_FOLLOWING
        self.direction = 0  # Current orientation
        self.node = "BoxInside"
        self.active = False
        self.last_button_press = 0

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
        """Interrupt handler for button press (start/stop/reset)."""
        # Debounce the button to prevent multiple triggers
        current_time = ticks_ms()
        if current_time - self.last_button_press < 500:  # 500ms debounce delay
            return
        self.last_button_press = current_time

        print(f"INTERRUPT: Button pressed on pin {pin}! Value: {pin.value()}")

        if self.is_reset:
            # First press: Start the robot
            self.stopped = False
            self.is_reset = False
            print("Robot STARTED")
        elif not self.stopped:
            # Second press (while running): Stop the robot
            self.stopped = True
            self.left_motor.off()
            self.right_motor.off()
            print("Robot STOPPED")
        else:
            # Third press (while stopped): Reset the program
            print("Resetting program...")
            raise KeyboardInterrupt("Button pressed - resetting robot")

    def update_node(self, new_node):
        """Update the current node of the robot."""
        self.node = new_node

        if new_node == "BoxInside":
            self.active = False
            self.yellow_light.value(1)
        else:
            self.active = True
            self.yellow_light.value(0)

    def line_follow(self, base_speed=80, correction_factor=15):
        """Follow a line using differential steering."""
        left_sensor = LineSensor.read(self.line_left_pin)
        right_sensor = LineSensor.read(self.line_right_pin)
        diff = (left_sensor - right_sensor) * correction_factor

        self.left_motor.speed(base_speed - diff)
        self.right_motor.speed(base_speed + diff)

    def line_follow_until_lost(self, base_speed=80, correction_factor=15):
        """Follow a line using differential steering."""
        while True:
            left_sensor = LineSensor.read(self.line_left_pin)
            right_sensor = LineSensor.read(self.line_right_pin)
            diff = (left_sensor - right_sensor) * correction_factor

            if left_sensor == 0 and right_sensor == 0:
                self.left_motor.off()
                self.right_motor.off()
                break

            self.left_motor.speed(base_speed - diff)
            self.right_motor.speed(base_speed + diff)

    def reverse_from_bay(self, speed=60):

        self.left_motor.speed(-speed)
        self.right_motor.speed(-speed)

        while LineSensor.read(self.junction_left_pin) == 0 or LineSensor.read(self.junction_right_pin) == 0:
            pass

        self.left_motor.off()
        self.right_motor.off()

        #turn out of bay - but move left wheel slower - so as to not hit the line on next bay - SHOULD BE FIXED MECHANICALLY
        start_time = ticks_ms()
        min_turn_time = 1000  # 1.5 seconds minimum turn time
        self.left_motor.speed(speed/1.5)
        self.right_motor.speed(-speed)
        while ticks_ms() - start_time < min_turn_time:
            pass
        while LineSensor.read(self.line_left_pin) == 0:
            pass

        self.direction = self.direction + 1

        self.left_motor.off()
        self.right_motor.off()

        while LineSensor.read(self.junction_left_pin) == 0 and LineSensor.read(self.junction_right_pin) == 0:
            self.line_follow(50, 30)

        self.left_motor.off()
        self.right_motor.off()



    def turn(self, rel_dir, speed=75):
        """
        Execute a turn based on relative direction.
        Updates the robot's direction state after turning.

        Args:
            rel_dir: Relative turn direction (+1=right, -1=left, 2=U-turn, 0=straight)
            speed: Motor speed during turn (default 75)
        """
        start_time = ticks_ms()
        min_turn_time = 750  # 1.5 seconds minimum turn time

        if rel_dir == 1:  # Turn right
            self.left_motor.speed(speed)
            self.right_motor.speed(-speed)
            while ticks_ms() - start_time < min_turn_time:
                pass
            while LineSensor.read(self.line_left_pin) == 0:
                pass
        elif rel_dir == -1:  # Turn left
            self.left_motor.speed(-speed)
            self.right_motor.speed(speed)
            while ticks_ms() - start_time < min_turn_time:
                pass
            while LineSensor.read(self.line_right_pin) == 0:
                pass
        elif rel_dir == 2:  # U-turn
            self.left_motor.speed(-speed)
            self.right_motor.speed(speed)
            while ticks_ms() - start_time < min_turn_time:
                pass
            while LineSensor.read(self.line_right_pin) == 0:
                pass

        self.left_motor.off()
        self.right_motor.off()

        # Update direction state based on relative turn
        self.direction = (self.direction + rel_dir) % 4

    def navigate_path(self, destination_node):
        """
        Navigate from the robot's current node to the destination node.
        Automatically finds the shortest path and updates the robot's current node.

        Args:
            destination_node: The target node name (string)
        """
        # Find the shortest path from current position to destination
        path = find_shortest_path(self.node, destination_node)

        if path is None:
            print(f"Error: Cannot find path from {self.node} to {destination_node}")
            return

        # Reset navigation state
        current_path_index = 0
        self.junction_detected = False
        self.robot_state = self.STATE_LINE_FOLLOWING

        print(f"Starting navigation: {path[0]} -> {path[-1]}")

        try:
            while current_path_index < len(path) - 1:

                print(f"State: {self.robot_state}, Path index: {current_path_index}/{len(path)-1}")

                # Check if stopped
                # if self.stopped:
                #     self.right_motor.off()
                #     self.left_motor.off()
                #     return

                if self.robot_state == self.STATE_LINE_FOLLOWING:
                    self._follow_line_to_junction(path, current_path_index)

                    # if self.stopped:
                    #     self.right_motor.off()
                    #     self.left_motor.off()
                    #     return

                    if self.junction_detected:
                        sleep(0.1)
                        current_path_index = self._handle_junction(path, current_path_index)

            print("Path navigation completed!")
            self.robot_state = self.STATE_COMPLETED

            # Update robot's current node to the destination
            self.update_node(destination_node)
            print(f"Robot now at node: {self.node}")

        except KeyboardInterrupt:
            print("Navigation interrupted by user")
            self.right_motor.off()
            self.left_motor.off()

        self.right_motor.off()
        self.left_motor.off()

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

        # Update robot's current node
        self.update_node(current_node)

        print(f"Junction detected! Now at node: {current_node}")

        # Check if there's a next node to navigate to
        if current_path_index < len(path) - 1:
            next_node = path[current_path_index + 1]
            new_dir = maze_map[current_node][next_node]
            rel = relative_turn(self.direction, new_dir)

            print(f"Navigation: {current_node} -> {next_node}")
            print(f"Direction change: {self.direction} -> {new_dir} (rel_turn={rel})")

            # Execute turn if needed (this will also update self.direction)
            if rel != 0:
                print(f"Executing turn: {rel}")
                self.turn(rel)
                if self.stopped:
                    return current_path_index
            else:
                print("No turn needed - going straight")


            self.left_motor.speed(80)
            self.right_motor.speed(80)
            sleep(0.1)
            self.left_motor.off()
            self.right_motor.off()

            print(f"Completed turn at {current_node}, resuming line following")

        # Reset junction detection
        self.junction_detected = False
        self.robot_state = self.STATE_LINE_FOLLOWING

        return current_path_index

