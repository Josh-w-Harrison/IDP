from machine import Pin, PWM
from time import sleep

# 1. Use the correct pin for Servo 1 [cite: 257]
pwm_pin = PWM(Pin(13))

# 2. Use the standard 50Hz frequency for servos [cite: 250, 254]
pwm_pin.freq(50)

# 3. Define the correct duty_u16 values based on the 500-2500us pulse spec
# (These are calculated for a 50Hz / 20,000us period)
MIN_DUTY = 1638  # 500us pulse
MID_DUTY = 4915  # 1500us pulse
MAX_DUTY = 8192  # 2500us pulse

# --- Test the servo ---

try:
    while True:
        print("Moving to MIN position (approx 0 deg)")
        pwm_pin.duty_u16(MIN_DUTY)
        sleep(2)

        print("Moving to MID position (approx 135 deg)")
        pwm_pin.duty_u16(MID_DUTY)
        sleep(2)

        print("Moving to MAX position (approx 270 deg)")
        pwm_pin.duty_u16(MAX_DUTY)
        sleep(2)

except KeyboardInterrupt:
    # This lets you stop the script in Thonny/PyCharm
    # It will turn off the PWM, stopping the servo
    pwm_pin.duty_u16(0)
    print("\nTest stopped. Servo off.")