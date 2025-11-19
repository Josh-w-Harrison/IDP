from machine import Pin


class LineSensor:
    """Reads digital line sensor values."""

    @staticmethod
    def read(pin_num):
        """Read a sensor value from a pin number."""
        return Pin(pin_num, Pin.IN, Pin.PULL_UP).value()

