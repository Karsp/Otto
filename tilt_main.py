import machine, time                       #importing machine and time libraries
from time import sleep                     #importing sleep class
from machine import Pin, ADC, PWM, SoftI2C #importing classes
from ottobuzzer import OttoBuzzer
from ottomotor import OttoMotor
from ottoneopixel import OttoUltrasonic
import random

tiltSensor = Pin(26, Pin.IN)	# Tilt sensor
buzzer = OttoBuzzer(25)               # Built in Buzzer
motor = OttoMotor(13, 14)             # Connectors 10 & 11
ultrasonic = OttoUltrasonic(18, 19)

offset = 0
tilt = 0
currentTime = time.gmtime()
prev = tiltSensor.value()

print(currentTime)

buzzer.playEmoji("S_happy")
motor.rightServo.duty(45+ offset)
motor.leftServo.duty(45- offset)
sleep(0.1)
motor.rightServo.duty(0)
motor.leftServo.duty(0)
motor.rightServo.duty(90+ offset)
motor.leftServo.duty(90- offset)
sleep(0.1)
motor.rightServo.duty(0)
motor.leftServo.duty(0)
ultrasonic.ultrasonicRGB1("ff9900", "ff9900")
while True:
    curr = tiltSensor.value()
    currentTime = time.gmtime()
    print(tilt)
    print(currentTime[5])
    if prev == (0) and curr == (1):
        buzzer.playNote(random.randint(100, 800), 125)
        tilt += 5
    prev = curr
    if (255) < tilt:
        tilt = 0
    ultrasonic.ultrasonicRGB2(0, 0, tilt)
