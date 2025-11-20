import sys
sys.path.append('src')
# sys.path.append('src/Controller')

from Controller.robot import Robot


# ------------------ MAIN ------------------

if __name__ == "__main__":
    # Create robot instance (starts at "BoxInside" by default)
    robot = Robot()

    # robot.turn(-1)

    # while True:
    #     robot.line_follow()

    print("Robot initialized. Press button to start/stop navigation.")
    print(f"Initial state: {'STOPPED' if robot.stopped else 'RUNNING'}")
    print(f"Starting node: {robot.node}")

    # Main control loop
    while True:
        if not robot.stopped:
            # Navigate to LowerRackA1 from current position
            robot.navigate_path("LowerRackA1")

            robot.turn(1)

            robot.line_follow_until_lost()

            robot.reverse_from_bay()

            #robot.turn(1)

            robot.navigate_path("BoxInside")

            robot.stopped = True  # Prevent immediate restart after completion
