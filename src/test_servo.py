from machine import Pin, PWM
from utime import sleep, ticks_ms, ticks_diff

def test_pwm():
    pwm_pin_no = 28  # GP28 (physical pin 34)
    pwm_pin = PWM(Pin(pwm_pin_no))
    pwm_pin.freq(100)

    level = 0       # 0–100 %
    direction = 1   # 1=up, -1=down
    start_time = ticks_ms()  # start timer

    while ticks_diff(ticks_ms(), start_time) < 10_000:  # run for 10 seconds
        u16_level = int(65535 * level / 100)
        pwm_pin.duty_u16(u16_level)

        print(f"Level={level}, u16_level={u16_level}, direction={direction}")
        level += direction
        if level == 100:
            direction = -1
        elif level == 0:
            direction = 1
        sleep(0.1)

    # after 10 seconds, turn PWM off
    pwm_pin.duty_u16(0)
    print("Test complete — PWM stopped.")

if __name__ == "__main__":
    test_pwm()
