from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo = 18, trigger = 22)
while True:
    print('Distance: ', sensor.distance * 100)
    sleep(1)