import machine, time                       #importing machine and time libraries
from time import sleep                     #importing sleep class
from machine import Pin, ADC, PWM, SoftI2C #importing classes
from ottobuzzer import OttoBuzzer
from ottomotor import OttoMotor
offset = 0
from ottoneopixel import OttoUltrasonic
from ottosensors import DHT

buzzer = OttoBuzzer(25)               # Built in Buzzer
motor = OttoMotor(13, 14)             # Connectors 10 & 11
ultrasonic = OttoUltrasonic(18, 19)
d = DHT(4)

buzzer.playEmoji("S_happy")
motor.rightServo.duty(45+ offset)
motor.leftServo.duty(45- offset)
sleep(0.1)
motor.rightServo.duty(0)
motor.leftServo.duty(0)
motor.rightServo.duty(115+ offset)
motor.leftServo.duty(115- offset)
sleep(0.1)
motor.rightServo.duty(0)
motor.leftServo.duty(0)
ultrasonic.ultrasonicRGB1("ff9900", "ff9900")
for count in range((10)):
    print("Otto is alive!")
    print(d.temperature())
    sleep((1))




import machine, time                       #importing machine and time libraries
from time import sleep                     #importing sleep class
from machine import Pin, ADC, PWM, SoftI2C #importing classes
from ottobuzzer import OttoBuzzer
from ottomotor import OttoMotor
offset = 0
from ottoneopixel import OttoUltrasonic
from ottosensors import DHT

buzzer = OttoBuzzer(25)               # Built in Buzzer
motor = OttoMotor(13, 14)             # Connectors 10 & 11
ultrasonic = OttoUltrasonic(18, 19)
d = DHT(4)
adc_27 = ADC(27)
adc_27.width(ADC.WIDTH_12BIT)
adc_27.atten(ADC.ATTN_11DB)
def sound(value, in_min, in_max, out_min, out_max):
   sound = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
   return sound

buzzer.playEmoji("S_happy")
motor.rightServo.duty(45+ offset)
motor.leftServo.duty(45- offset)
sleep(0.1)
motor.rightServo.duty(0)
motor.leftServo.duty(0)
motor.rightServo.duty(115+ offset)
motor.leftServo.duty(115- offset)
sleep(0.1)
motor.rightServo.duty(0)
motor.leftServo.duty(0)
ultrasonic.ultrasonicRGB1("ff9900", "ff9900")
for count in range((10)):
    print("Otto is alive!")
    print(d.temperature())
    if int(sound(adc_27.read(), 0, 4095, 0, 100)):
        print(int(sound(adc_27.read(), 0, 4095, 0, 100)))
    sleep((1))
