import sys
sys.path.append('src')

from Controller.motor import Motor
from Controller.actuator import Actuator
from Controller.line_sensor import LineSensor
from Controller.robot import Robot
from Controller.navigation import find_shortest_path


# ------------------ MAIN ------------------

if __name__ == "__main__":
    # Define navigation path
    start_node = "BoxInside"
    end_node = "UpperRackA3"
    
    # Find shortest path
    path = find_shortest_path(start_node, end_node)
    print(f"Path: {' -> '.join(path)}")

    # Create robot instance
    robot = Robot()

    print("Robot initialized. Press button to start/stop navigation.")
    print(f"Initial state: {'STOPPED' if robot.stopped else 'RUNNING'}")

    # Main control loop
    while True:
        if not robot.stopped:
            robot.navigate_path(path)
            robot.stopped = True  # Prevent immediate restart after completion
