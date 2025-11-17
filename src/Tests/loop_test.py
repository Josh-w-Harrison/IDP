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
import junctions_line_following


base_speed = 60
turn_speed = 35



def manual_navigation():
    motor_left = actuator_class.Motor(dirPin=4, PWMPin=5)
    motor_right = actuator_class.Motor(dirPin=7, PWMPin=6)

    print("Starting manual navigation sequence...")

    # Step 1: Forward 0.4 m (time-based as before)
    state_machine.move_forward(motor_left, motor_right, speed=70, distance_m=0.55)

    # Step 3: LINE FOLLOWING instead of blind forward 1.1 m
    print("STEP 3: line-following for ~3.0 m")
    junctions_line_following.line_following(motor_left, motor_right, base_speed=60, soft_turn_speed=35, hard_turn_speed=25)

    print("Sequence complete.")


if __name__ == "__main__":
    manual_navigation()