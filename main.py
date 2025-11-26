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
    
    colour_sensor = TCS34725(i2c)
    
    distance_sensor = VL53L0X(i2c_1)

    print("Robot initialized. Press button to start/stop navigation.")

    while True:
        if not robot.stopped:
            
            rack_nodes = [
                "LowerRackA6",
                "LowerRackA5",
                "LowerRackA4",
                "LowerRackA3",
                "LowerRackA2",
                "LowerRackA1"
                ]
            
            for node in rack_nodes:
            
                robot.navigate_path(node)
                robot.turn_abs(1)
                robot.line_follow_until_lost()
                
                if distance_sensor.read() < 100:
                    box = True
                    box_colour = colour_sensor.get_color()
                else:
                    box = False
                
                robot.reverse_from_bay()
                
                if box:
                    robot.turn_abs(2)
                    robot.navigate_path(box_colour)
                    robot.turn_abs(2)
                    robot.line_follow_until_lost()
                    robot.reverse_from_bay()
                    if box_colour == "BlueJunction":
                        robot.turn_abs(0)
                    else:
                        robot.turn_abs(3)
                    robot.navigate_path(node)
                else:
                    robot.turn_abs(0)
            

            
            robot.stopped = True  # Prevent immediate restart after completion
