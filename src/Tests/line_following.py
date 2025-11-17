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


# Return value of each sensor once per call (no infinite loop)
def read_sensor(pin_num):
    sensor = Pin(pin_num, Pin.IN, Pin.PULL_UP)  # <-- use pull-up instead
    return sensor.value()  # 1=white (on line), 0=black (off line)


def easy_line_following():
    motor_left = actuator_class.Motor(dirPin=4, PWMPin=5)
    motor_right = actuator_class.Motor(dirPin=7, PWMPin=6)

    base_speed = 60
    turn_speed = 35

    print("Starting line following...")

    while True:
        # Read all sensors
        s16 = read_sensor(16)
        s17 = read_sensor(17)
        s18 = read_sensor(18)
        s19 = read_sensor(19)

        print(f"Sensors: 16={s16}, 17={s17}, 18={s18}, 19={s19}")

        # Logic: 1 = white (on line), 0 = black (off line)
        if s17 == 1 and s18 == 1:
            # On the line — go forward
            print("CENTERED Forward")
            motor_left.Forward(base_speed)
            motor_right.Forward(base_speed)

        elif s17 == 0 and s18 == 1:
            # Line drifting left → turn right
            print("LEFT EDGE Turn right")
            motor_right.Forward(turn_speed)
            motor_left.Forward(base_speed)

        elif s17 == 1 and s18 == 0:
            # Line drifting right → turn left
            print("RIGHT EDGE Turn left")
            motor_right.Forward(base_speed)
            motor_left.Forward(turn_speed)

        else:
            # Lost line — stop
            print("LOST LINE Stop")
            motor_left.off()
            motor_right.off()

        sleep(0.1)


if __name__ == "__main__":
    easy_line_following()