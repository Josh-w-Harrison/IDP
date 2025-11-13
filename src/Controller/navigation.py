# Code for the manual navigation around the maze 

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

def grab():
    print("GRAB closing gripper")
    # Replace with servo or electromagnet control later
    sleep(1)


def release():
    print("RELEASE opening gripper")
    sleep(1)


def manual_navigation():
    motor_left = Motor(dirPin=4, PWMPin=5)
    motor_right = Motor(dirPin=7, PWMPin=6)

    print("Starting manual navigation sequence...")

    # Step 1: Forward 0.2 m
    move_forward(motor_left, motor_right, speed=60, distance_m=0.4)
    sleep(100)

    # Step 2: Turn left (1.0 s)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    # Step 3: Go forward 1.1 m
    move_forward(motor_left, motor_right, speed=60, distance_m=1.1)

    # Step 4: Turn right (use turning speed)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)

    # Step 5: Go forward 0.1 m
    move_forward(motor_left, motor_right, speed=60, distance_m=0.1)


    # PICKING UP THE FIRST BLOCK FROM THE RACK A

    # Step 1: Turn right (align to rack)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 2: Go forward until stop (approx. 0.25 m)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.25)
    # Step 3: Grab block
    grab()
    # Step 4: Go reverse (back out)
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.25)
    # Step 5: Turn left relative to reverse (so effectively same orientation as before)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 6: Go forward (return toward slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.4)
    # Step 7: Turn right (align to slot corridor)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 8: Go forward (approach slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.25)
    # Step 9: Place block into correct color slot
    release()
    # Step 10: Go reverse
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.25)
    # Step 11: Turn right relative to reverse (ready to move to next rack)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    print("Block 1 done.")

    # Step 1: Turn right (align to rack)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 2: Go forward until stop (approx. 0.35 m)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.35)
    # Step 3: Grab block
    grab()
    # Step 4: Go reverse (back out)
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.35)
    # Step 5: Turn left relative to reverse (so effectively same orientation as before)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 6: Go forward (return toward slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.4)
    # Step 7: Turn right (align to slot corridor)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 8: Go forward (approach slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.35)
    # Step 9: Place block into correct color slot
    release()
    # Step 10: Go reverse
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.35)
    # Step 11: Turn right relative to reverse (ready to move to next rack)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    print("Block 2 done.")

    # Step 1: Turn right (align to rack)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 2: Go forward until stop (approx. 0.45 m)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.45)
    # Step 3: Grab block
    grab()
    # Step 4: Go reverse (back out)
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.45)
    # Step 5: Turn left relative to reverse (so effectively same orientation as before)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 6: Go forward (return toward slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.45)
    # Step 7: Turn right (align to slot corridor)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 8: Go forward (approach slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.45)
    # Step 9: Place block into correct color slot
    release()
    # Step 10: Go reverse
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.45)
    # Step 11: Turn right relative to reverse (ready to move to next rack)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    print("Block 3 done.")

    # Step 1: Turn right (align to rack)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 2: Go forward until stop (approx. 0.55 m)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.55)
    # Step 3: Grab block
    grab()
    # Step 4: Go reverse (back out)
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.55)
    # Step 5: Turn left relative to reverse (so effectively same orientation as before)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 6: Go forward (return toward slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.55)
    # Step 7: Turn right (align to slot corridor)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 8: Go forward (approach slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.55)
    # Step 9: Place block into correct color slot
    release()
    # Step 10: Go reverse
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.55)
    # Step 11: Turn right relative to reverse (ready to move to next rack)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    print("Block 4 done.")

    # Step 1: Turn right (align to rack)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 2: Go forward until stop (approx. 0.65 m)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.65)
    # Step 3: Grab block
    grab()
    # Step 4: Go reverse (back out)
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.65)
    # Step 5: Turn left relative to reverse (so effectively same orientation as before)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 6: Go forward (return toward slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.65)
    # Step 7: Turn right (align to slot corridor)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 8: Go forward (approach slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.65)
    # Step 9: Place block into correct color slot
    release()
    # Step 10: Go reverse
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.65)
    # Step 11: Turn right relative to reverse (ready to move to next rack)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    print("Block 5 done.")

    # Step 1: Turn right (align to rack)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 2: Go forward until stop (approx. 0.75 m)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.75)
    # Step 3: Grab block
    grab()
    # Step 4: Go reverse (back out)
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.75)
    # Step 5: Turn left relative to reverse (so effectively same orientation as before)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 6: Go forward (return toward slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.75)
    # Step 7: Turn right (align to slot corridor)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 8: Go forward (approach slot)
    move_forward(motor_left, motor_right, speed=60, distance_m=0.75)
    # Step 9: Place block into correct color slot
    release()
    # Step 10: Go reverse
    move_reverse(motor_left, motor_right, speed=60, distance_m=0.75)
    # Step 11: Turn right relative to reverse (ready to move to next rack)
    turn_right(motor_left, motor_right, turning_speed=40, turn_time=1.0)

    print("Block 6 done.")



    # Going from the original position of the robot to rack A UPPER PART

    # Step 1: Forward 0.2 m
    move_forward(motor_left, motor_right, speed=60, distance_m=0.4)
    # Step 2: Turn right (1.0 s)
    turn_left(motor_left, motor_right, turning_speed=40, turn_time=1.0)
    # Step 3: Go forward 1.1 m
    move_forward(motor_left, motor_right, speed=60, distance_m=1.1)
    # Step 4: Turn left (use turning speed)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 5: Go forward 0.1 m
    move_forward(motor_left, motor_right, speed=70, distance_m=0.1)
    # Step 6: Turn left (use turning speed)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 4: Turn left (use turning speed)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    # Step 5: Go forward 0.1 m
    move_forward(motor_left, motor_right, speed=70, distance_m=0.1)
    # Step 6: Turn left (use turning speed)
    turn_right(motor_left, motor_right, turning_speed=35, turn_time=1.0)
    move_forward(motor_left, motor_right, speed=70, distance_m=0.1)
    



    print("At the upper section of the track.")



    # Picking up the blocks from the UPPER PART OF THE RACK A 







    print("Sequence complete.")


if __name__ == "__main__":
    manual_navigation()