from gpiozero import Robot, DigitalInputDevice
from time import sleep

class QuadratureEncoder(object):
    """
    A simple quadrature encoder class
    """
    def __init__(self, pin_a, pin_b):
        self._a_value = 0
        self._b_value = 0

        encoder_a = DigitalInputDevice(pin_a)
        encoder_a.when_activated = self._increment_a
        encoder_a.when_deactivated = self._increment_a

        encoder_b = DigitalInputDevice(pin_b)
        encoder_b.when_activated = self._increment_b
        encoder_b.when_deactivated = self._increment_b
        
    def reset(self):
        self._a_value = 0
        self._b_value = 0

    def _increment_a(self):
        self._a_value += 1

    def _increment_b(self):
        self._b_value += 1

    @property
    def a_value(self):
        return self._a_value
    @property
    def b_value(self):
        return self._b_value

r = Robot((27, 17), (23, 24))
e1 = QuadratureEncoder(13, 19)
# e2 = QuadratureEncoder(18, 19)

r.value = (1, 1)

while True:
    print("e1, a: {} b: {}".format(e1.a_value, e1.b_value))
    sleep(1)