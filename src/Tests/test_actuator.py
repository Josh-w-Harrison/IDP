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


def test_actuator1():
    actuator1 = actuator_class.Actuator(dirPin=0, PWMPin=1)  # Actuator 1 controlled from Motor Driv1 #1, which is on GP0/1

    while True:
        print("Extending quickly")
        actuator1.set(dir = 0, speed=100)
        sleep(5)  # nb we don't know when this has finished without another means

        print("Retracing slowly")
        actuator1.set(dir=1, speed=25)
        sleep(10)  # nb we don't know when this has finished without another means


if __name__ == "__main__":
    test_actuator1()