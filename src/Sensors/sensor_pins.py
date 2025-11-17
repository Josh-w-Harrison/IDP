from machine import Pin
from utime import sleep


def read_sensor(pin_num):
    sensor = Pin(pin_num, Pin.IN, Pin.PULL_UP)
    return sensor.value()   # 1 = white, 0 = black  (adjust if needed)

def get_line_binary():
    left_junction_sensor = read_sensor(8)  # leftmost
    left_line_sensor = read_sensor(9)
    right_line_sensor = read_sensor(10)
    right_junction_sensor = read_sensor(11)  # rightmost

    return (right_junction_sensor << 3) | (right_line_sensor << 2) | (left_line_sensor << 1) | left_junction_sensor


left_junction_sensor = read_sensor(8)
left_line_sensor = read_sensor(9)
right_line_sensor = read_sensor(10)
right_junction_sensor = read_sensor(11)