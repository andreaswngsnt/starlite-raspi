from gpiozero import Robot, DigitalInputDevice, RotaryEncoder
from time import sleep

class QuadratureEncoder(object):
    """
    A simple quadrature encoder class
    """
    def __init__(self, pin_a, pin_b):
        self._a_value = 0
        self._b_value = 0

        self.encoder_a = DigitalInputDevice(pin_a)
        self.encoder_a.when_activated = self._increment_a
        self.encoder_a.when_deactivated = self._increment_a

        self.encoder_b = DigitalInputDevice(pin_b)
        self.encoder_b.when_activated = self._increment_b
        self.encoder_b.when_deactivated = self._increment_b
        
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
# e1 = RotaryEncoder(13, 19, wrap = True, max_steps = 180)

r.value = (0.1, 0.1)

# The encoder has 990 * 2 counts / rotation
# Will rotate 5 times
while e1.a_value <= 990 * 2 * 5:
    print("e1, a: {} b: {}".format(e1.a_value, e1.b_value))
    sleep(1)

print("e1, a: {} b: {}".format(e1.a_value, e1.b_value))

# while e1.value * 180 < 90:
#     print("e1, {}, {}".format(e1.value, e1.value * 180))
#     sleep(0.5)