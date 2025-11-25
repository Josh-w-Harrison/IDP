from machine import Pin, I2C
import time

from robot import Robot
from sensors import TCS34725, VL53L0X

# ------------------ MAIN ------------------

if __name__ == "__main__":
    robot = Robot()
    
    power_on= Pin(22, Pin.OUT)
    power_on.value(1)
    time.sleep(1)
    i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
    print(i2c.scan())
    
    i2c_1 = I2C(1, sda=Pin(18), scl=Pin(19))
    
    # sensor = TCS34725(i2c)
    
    # distance_sensor = VL53L0X(i2c_1)

    print("Robot initialized. Press button to start/stop navigation.")

    while True:
        if not robot.stopped:
            
            robot.navigate_path("UpperRackB6")

            robot.turn(1)
            
            robot.line_follow_until_lost()
            
            # if distance_sensor.read() < 200:
            
            #     box_colour = sensor.get_color()
            #     print(box_colour)
                
            robot.reverse_from_bay()
            
            robot.turn(1)
            
            robot.navigate_path("RedJunction")
            
            robot.line_follow_until_lost()
            
            robot.reverse_from_bay()
            
            robot.turn(1)
            
            robot.navigate_path("BoxInside")
            
            robot.stopped = True  # Prevent immediate restart after completion
