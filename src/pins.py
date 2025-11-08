# Raspberry Pi Pico W, typed this from the .pdf file on Moodle with all the pins for the board 

PINS = {
    # MOTORS 
    "MOTOR_LEFT_FWD": 2,     # GP2  (Pin 4)
    "MOTOR_LEFT_REV": 3,     # GP3  (Pin 5)
    "MOTOR_RIGHT_FWD": 4,    # GP4  (Pin 6)
    "MOTOR_RIGHT_REV": 5,    # GP5  (Pin 7)

    # SERVO
    "SERVO": 6,              # GP6  (Pin 9)

    # LED INDICATORS
    "LED_AMBER": 7,          # GP7  (Pin 10)
    "LED_RED": 8,            # GP8  (Pin 11)

    # BUTTON (Start/Stop)
    "BUTTON_START": 9,       # GP9  (Pin 12)

    # LINE SENSORS
    "LINE_LEFT": 26,         # GP26 / ADC0 (Pin 31)
    "LINE_MID": 27,          # GP27 / ADC1 (Pin 32)
    "LINE_RIGHT": 28,        # GP28 / ADC2 (Pin 34)

    # COLOUR SENSOR (TCS34725)
    # Default I²C1 bus on GP20 (SDA) / GP21 (SCL)
    "COLOR_SDA": 20,         # GP20 (Pin 26)
    "COLOR_SCL": 21,         # GP21 (Pin 27)

    # ULTRASONIC SENSOR (HC-SR04)
    "ULTRA_TRIG": 14,        # GP14 (Pin 19)
    "ULTRA_ECHO": 15,        # GP15 (Pin 21)

    # POWER PINS (for reference only)
    "VCC_3V3": 36,           # 3V3(OUT)
    "VCC_5V": 40,            # VBUS
    "GND": [3, 8, 13, 18, 23, 28, 33, 38]  # All ground pins
}
