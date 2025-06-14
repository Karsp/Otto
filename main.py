# /* ········································································································································· */
# /* :                                                                                                                                       : */
# /* :                                                                                                                                       : */
# /* :                                                                                                                                       : */
# /* :               _____                    _____                    _____                    _____                    _____               : */
# /* :              /\    \                  /\    \                  /\    \                  /\    \                  /\    \              : */
# /* :             /::\____\                /::\    \                /::\    \                /::\    \                /::\    \             : */
# /* :            /:::/    /               /::::\    \              /::::\    \              /::::\    \              /::::\    \            : */
# /* :           /:::/    /               /::::::\    \            /::::::\    \            /::::::\    \            /::::::\    \           : */
# /* :          /:::/    /               /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \          /:::/\:::\    \          : */
# /* :         /:::/____/               /:::/__\:::\    \        /:::/__\:::\    \        /:::/__\:::\    \        /:::/__\:::\    \         : */
# /* :        /::::\    \              /::::\   \:::\    \      /::::\   \:::\    \       \:::\   \:::\    \      /::::\   \:::\    \        : */
# /* :       /::::::\____\________    /::::::\   \:::\    \    /::::::\   \:::\    \    ___\:::\   \:::\    \    /::::::\   \:::\    \       : */
# /* :      /:::/\:::::::::::\    \  /:::/\:::\   \:::\    \  /:::/\:::\   \:::\____\  /\   \:::\   \:::\    \  /:::/\:::\   \:::\____\      : */
# /* :     /:::/  |:::::::::::\____\/:::/  \:::\   \:::\____\/:::/  \:::\   \:::|    |/::\   \:::\   \:::\____\/:::/  \:::\   \:::|    |     : */
# /* :     \::/   |::|~~~|~~~~~     \::/    \:::\  /:::/    /\::/   |::::\  /:::|____|\:::\   \:::\   \::/    /\::/    \:::\  /:::|____|     : */
# /* :      \/____|::|   |           \/____/ \:::\/:::/    /  \/____|:::::\/:::/    /  \:::\   \:::\   \/____/  \/_____/\:::\/:::/    /      : */
# /* :            |::|   |                    \::::::/    /         |:::::::::/    /    \:::\   \:::\    \               \::::::/    /       : */
# /* :            |::|   |                     \::::/    /          |::|\::::/    /      \:::\   \:::\____\               \::::/    /        : */
# /* :            |::|   |                     /:::/    /           |::| \::/____/        \:::\  /:::/    /                \::/____/         : */
# /* :            |::|   |                    /:::/    /            |::|  ~|               \:::\/:::/    /                  ~~               : */
# /* :            |::|   |                   /:::/    /             |::|   |                \::::::/    /                                    : */
# /* :            \::|   |                  /:::/    /              \::|   |                 \::::/    /                                     : */
# /* :             \:|   |                  \::/    /                \:|   |                  \::/    /                                      : */
# /* :              \|___|                   \/____/                  \|___|                   \/____/                                       : */
# /* :                                                                                                                                       : */
# /* :                                                                                                                                       : */
# /* :                                                                                                                                       : */
# /* ········································································································································· */

import machine, time
from time import sleep
from machine import Pin, ADC, PWM, SoftI2C

from ottobuzzer   import OttoBuzzer         # sound
from ottomotor    import OttoMotor          # servos
from ottoneopixel import OttoNeoPixel       # LED ring
from ottoneopixel import OttoUltrasonic     # “eye” LEDs + ultrasonic
from ottosensors  import DHT     # sense-kit modules
# …or NeoPixel/Player/Ninja modules as needed

buzzer    = OttoBuzzer(25)
motor     = OttoMotor(13, 14)
ring      = OttoNeoPixel(4, 13)
ultrasonic= OttoUltrasonic(18, 19)


# 1) Initialize
buzzer    = OttoBuzzer(25)
motor     = OttoMotor(13, 14)
ring      = OttoNeoPixel(4, 13)
ultrasonic= OttoUltrasonic(18, 19)

# 2) Hardware tests
buzzer.playEmoji("S_connection")

# ring.rainbow_cycle(13, 2)
ultrasonic.ultrasonicRGB1("00ff00","00ff00")

motor.Moveleft(-1,1,1)
motor.Moveright(-1,1,1)

# 3) Main loop: obstacle warning
# while True:
#     dist = int(ultrasonic.readDistance())
#     if dist <= 10:
#         ring.fillAllRGBRing("fe0000")
#         buzzer.playEmoji("S_surprise")
#     else:
#         ring.fillAllRGBRing("33ff33")


from hummingbird import *
import time

# Print brightness value from the light sensor every second.
while True:
    brightness = get_light_sensor(1)
    print("Brightness:", brightness)
    time.sleep(1)


# Print distance in cm, stop if object is too close.
# while True:
#     distance = get_distance_sensor(1)
#     print("Distance (cm):", distance)
#     if distance < 10:
#         print("Too close! Stopping.")
#         break
#     time.sleep(0.5)
