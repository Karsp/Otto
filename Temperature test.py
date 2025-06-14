# Temperature test
import machine, time                       #importing machine and time libraries
from time import sleep                     #importing sleep class
from machine import Pin, ADC, PWM, SoftI2C #importing classes
from ottosensors import DHT

d = DHT(4)

for count in range((10)):
    print("Otto is alive!")
    print(d.temperature())
    sleep((1))
