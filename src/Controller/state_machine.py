import sys
sys.path.append('src/Actuators')
sys.path.append('src/Sensors')
sys.path.append('src/Controller')

from machine import Pin, PWM
from utime import sleep, ticks_ms
import Tests.junctions_line_following as junctions_line_following
from Sensors import sensor_pins
from Actuators import actuator_class
from Controller import state_machine

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
    print(f"REVERSE {distance_m:.2f} m (≈ {time_s:.2f} s)")
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


def line_following():

    base_speed = 60
    turn_speed = 35

    print("Starting line following...")

    while True:
        # Read all sensors
        s16 = sensor_pins.read_sensor(16)
        s17 = sensor_pins.read_sensor(17)
        s18 = sensor_pins.read_sensor(18)
        s19 = sensor_pins.read_sensor(19)

        print(f"Sensors: 16={s16}, 17={s17}, 18={s18}, 19={s19}")

        # Logic: 1 = white (on line), 0 = black (off line)
        if s17 == 1 and s18 == 1:
            # On the line — go forward
            print("CENTERED Forward")
            actuator_class.motor_left.Forward(base_speed)
            actuator_class.motor_right.Forward(base_speed)

        elif s17 == 0 and s18 == 1:
            # Line drifting left → turn right
            print("LEFT EDGE Turn right")
            actuator_class.motor_left.Forward(turn_speed)
            actuator_class.motor_right.Forward(base_speed)

        elif s17 == 1 and s18 == 0:
            # Line drifting right → turn left
            print("RIGHT EDGE Turn left")
            actuator_class.motor_left.Forward(base_speed)
            actuator_class.motor_right.Forward(turn_speed)

        else:
            # Lost line — stop
            print("LOST LINE Stop")
            actuator_class.motor_left.off()
            actuator_class.motor_right.off()

        sleep(0.1)
