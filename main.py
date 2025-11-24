from machine import Pin, I2C
import time

import sys
sys.path.append('src')
# sys.path.append('src/Controller')

# from Controller.robot import Robot
from robot import Robot
from sensors import TCS34725, vl53l0x


# ------------------ MAIN ------------------

if __name__ == "__main__":
    robot = Robot()
    
    power_on= Pin(22, Pin.OUT)
    power_on.value(1)
    time.sleep(1)
    i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
    print(i2c.scan())
    
    sensor = TCS34725(i2c)

    print("Robot initialized. Press button to start/stop navigation.")

    while True:
        if not robot.stopped:
            robot.navigate_path("UpperRackB6")

            robot.turn(1)
            
            if vl530l0x() < 200:

                robot.line_follow_until_lost()
            
                clear, red, green, blue = sensor.read_raw()
                temp = sensor.calculate_color_temperature(red, green, blue)
                lux = sensor.calculate_lux(red, green, blue)

                print("Clear: {}, Red: {}, Green: {}, Blue: {}".format(clear, red, green, blue))
                print("Color Temp: {} K, Lux: {}".format(temp, lux))
                print("-" * 40)

                rn, gn, bn = sensor.normalize(red, green, blue, clear)
                print('Classified color')
                box_colour = sensor.classify_color(rn, gn, bn, temp)

                robot.reverse_from_bay()

                robot.turn(1)

                robot.navigate_path(box_colour)
                
            robot.navigate_path("BoxInside")

            robot.stopped = True  # Prevent immediate restart after completion
