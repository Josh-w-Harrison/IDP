from machine import Pin
from utime import sleep


# LINE SENSOR FUNCTION PROVIDED BY THE CUED GITHUB REPO 
def test_input_poll():
    "Simple poll of input"
    input_pin = 19  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        return value
        sleep(0.2)





#FUNCTIONS FOR THE LINE SENSORS

# First initialise the pins in which we are planning to mount the line sensors 

def line_sensor_16():
    "Simple poll of input"
    input_pin = 16  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)

def line_sensor_17():
    "Simple poll of input"
    input_pin = 17  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)


def line_sensor_18():
    "Simple poll of input"
    input_pin = 18  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)

def line_sensor_19():
    "Simple poll of input"
    input_pin = 19  # Pin 19 = GP19 (labelled 23 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.value()
        print(f"Input = {value}")
        sleep(0.2)





# WE STILL THINK ITS ALWAYS BEST TO BE WORKING WITH NO INTERRUPTS AND INCLUDE ALL POSSIBLE DATA IN THE ANALYSIS YET 




def input_irq(p):
    "Interrupt handler"
    # print(p)
    value = p.value()
    print(f"Input changed, value={value}")


def test_input_irq():
    "More advanced, interrupt based input handling"
    input_pin = 18  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down
    input.irq(handler=input_irq) # Register irq, you could also consider rising and falling edges c.f. https://docs.micropython.org/en/latest/library/machine.Pin.html

    while True:
        pass # irq handling does the rest in this instance


if __name__ == "__main__":
    test_input_poll()
    # test_input_irq()