Examples

Below are sets of 5 ascending-difficulty examples for each kit category. Each set’s first 3 examples use only components from that kit; examples 4–5 combine with other compatible modules (respecting the noted hardware conflicts). Each example has a short description, playful tone, and a MicroPython-like snippet.

---

## A. STANDARD KIT (Two motors, color light ring, ultrasonic sensor (distance + “eyes”), eyes color light, line-follow sensor)

> **Note:** In combined examples, we avoid using the RGB light ring if also using Sense/Interact modules simultaneously (per compatibility). We treat “eyes color light” and “ultrasonic distance” as separate from the ring, but when combining with Sense/Interact, we’ll avoid ring usage.

### 1. Easy: “Wheel Check + Hello Ring”

**Goal:** Verify both motors spin and ring lights up in a simple pattern.

```python
from time import sleep
from ottomotor import OttoMotor
from ottoneopixel import OttoNeoPixel

motor = OttoMotor(13, 14)
ring = OttoNeoPixel(4, 13)

# Spin forward then backward
motor.leftServo.freq(50); motor.rightServo.freq(50)
print("Testing forward")
motor.leftServo.duty(95); motor.rightServo.duty(43)
sleep(1)
motor.leftServo.duty(0); motor.rightServo.duty(0)
sleep(0.5)
print("Testing backward")
motor.leftServo.duty(43); motor.rightServo.duty(95)
sleep(1)
motor.leftServo.duty(0); motor.rightServo.duty(0)

# Light ring rainbow
print("Rainbow ring!")
ring.setBrightness(0.5)
ring.rainbow_cycle(13, 1)  # 1 second speed
```

> **Tone:** Friendly “let’s check the wheels and ring.”
> **Uses:** Motors + ring.

---

### 2. Medium: “Line-Dance”

**Goal:** Drive forward until a line is detected, then spin in place with colored ring blink.

```python
from machine import ADC, Pin
from ottomotor import OttoMotor
from ottoneopixel import OttoNeoPixel
from time import sleep

motor = OttoMotor(13, 14)
ring  = OttoNeoPixel(4, 13)
analogL = ADC(Pin(32))  # left line sensor
analogR = ADC(Pin(33))  # right line sensor
analogL.width(ADC.WIDTH_12BIT); analogL.atten(ADC.ATTN_11DB)
analogR.width(ADC.WIDTH_12BIT); analogR.atten(ADC.ATTN_11DB)

motor.leftServo.freq(50); motor.rightServo.freq(50)
ring.setBrightness(0.6)

print("Starting forward until line is found")
while True:
    # Move forward
    motor.leftServo.duty(95); motor.rightServo.duty(43)
    valL = analogL.read(); valR = analogR.read()
    # Suppose >800 means black line
    if valL > 800 or valR > 800:
        motor.leftServo.duty(0); motor.rightServo.duty(0)
        print("Line detected! Spinning dance.")
        # Spin in place with ring blink
        for _ in range(4):
            motor.leftServo.duty(95); motor.rightServo.duty(95)  # turn right
            ring.fillAllRGBRing("ff00ff")
            sleep(0.3)
            motor.leftServo.duty(0); motor.rightServo.duty(0)
            ring.fillAllRGBRing("00ffff")
            sleep(0.3)
        break
    sleep(0.05)
motor.leftServo.duty(0); motor.rightServo.duty(0)
print("Line-Dance complete")
```

> **Tone:** Encouraging “dance when you see the line.”
> **Uses:** Motors + line sensor + ring.

---

### 3. Intermediate: “Guard Duty”

**Goal:** Patrol forward/backward; if object <15cm, flash eyes red and back up; else green eyes.

```python
from ottomotor import OttoMotor
from ottoneopixel import OttoUltrasonic
from time import sleep

motor = OttoMotor(13, 14)
eyes  = OttoUltrasonic(18, 19)  # supports reading distance and setting eye color
motor.leftServo.freq(50); motor.rightServo.freq(50)

def flash_eyes(color, times=3):
    for _ in range(times):
        eyes.ultrasonicRGB1(color, color)
        sleep(0.3)
        eyes.clearultrasonicRGB()
        sleep(0.2)

print("Starting Guard Duty patrol")
while True:
    dist = int(eyes.readDistance())  # assume readDistance() gives cm
    if dist < 15:
        print("Intruder! Backing up")
        flash_eyes("ff0000")
        motor.leftServo.duty(43); motor.rightServo.duty(109)  # back up
        sleep(0.5)
        motor.leftServo.duty(0); motor.rightServo.duty(0)
    else:
        eyes.ultrasonicRGB1("00ff00", "00ff00")  # green eyes
        # patrol forward a bit
        motor.leftServo.duty(95); motor.rightServo.duty(43)
        sleep(0.4)
        motor.leftServo.duty(0); motor.rightServo.duty(0)
        # then reverse a bit
        motor.leftServo.duty(43); motor.rightServo.duty(109)
        sleep(0.4)
        motor.leftServo.duty(0); motor.rightServo.duty(0)
    sleep(0.2)
```

> **Tone:** Heroic “guard duty,” simple state behavior.
> **Uses:** Motors + ultrasonic eyes.

---

### 4. Harder: “Sound-Triggered Evade” (Standard + Sense’s microphone)

**Goal:** When a clap is heard via microphone (Sense expansion), Otto spins and flashes eyes, then resumes patrol (ultrasonic). *Note:* ring not used here since using Sense mic.

```python
from machine import ADC, Pin
from ottomotor import OttoMotor
from ottoneopixel import OttoUltrasonic
from time import sleep

# Sense: microphone on ADC pin 32
adc_mic = ADC(Pin(32))
adc_mic.width(ADC.WIDTH_12BIT); adc_mic.atten(ADC.ATTN_11DB)

motor = OttoMotor(13, 14)
eyes  = OttoUltrasonic(18, 19)
motor.leftServo.freq(50); motor.rightServo.freq(50)

def read_sound_level():
    val = adc_mic.read()
    # map 0–4095 to 0–100
    return (val * 100) // 4095

print("Starting Sound-Triggered Evade patrol")
while True:
    # Check for clap/loud sound
    sound_lvl = read_sound_level()
    if sound_lvl > 60:
        print("Clap detected! Evading...")
        # spin with red eyes
        for _ in range(3):
            eyes.ultrasonicRGB1("ff0000","ff0000")
            motor.leftServo.duty(95); motor.rightServo.duty(95)  # spin
            sleep(0.3)
            motor.leftServo.duty(0); motor.rightServo.duty(0)
            eyes.clearultrasonicRGB()
            sleep(0.2)
        continue  # go back to patrol
    # Patrol with ultrasonic as before
    dist = int(eyes.readDistance())
    if dist < 15:
        eyes.ultrasonicRGB1("ff8800","ff8800")
        motor.leftServo.duty(43); motor.rightServo.duty(109)
        sleep(0.5)
        motor.leftServo.duty(0); motor.rightServo.duty(0)
    else:
        eyes.ultrasonicRGB1("00ff00","00ff00")
        motor.leftServo.duty(95); motor.rightServo.duty(43)
        sleep(0.3)
        motor.leftServo.duty(0); motor.rightServo.duty(0)
    sleep(0.1)
```

> **Tone:** Reactive “clap makes me dodge,” combining standard motors/ultrasonic + microphone.
> **Uses:** Motors + ultrasonic eyes + microphone (Sense).

---

### 5. Hard: “Tilt-Activated Rescue” (Standard + Sense tilt + Interact button)

**Goal:** Otto patrols; if robot is tilted (Sense tilt sensor), enters “rescue mode”: flashes eyes, waits for user to press a physical button (Interact kit) to “stabilize,” then resumes. Ring not used to avoid conflict if sense/interact plugged.

```python
from machine import Pin, ADC
from ottomotor import OttoMotor
from ottoneopixel import OttoUltrasonic
import utime
from ottosensors import DHT  # reused if needed
# Sense tilt: assume digital tilt sensor on pin 26
tilt_pin = Pin(26, Pin.IN)
# Interact: push button on pin 27
btn = Pin(27, Pin.IN, Pin.PULL_UP)

motor = OttoMotor(13, 14)
eyes  = OttoUltrasonic(18, 19)
motor.leftServo.freq(50); motor.rightServo.freq(50)

def patrol_step():
    dist = int(eyes.readDistance())
    if dist < 15:
        eyes.ultrasonicRGB1("ff0000","ff0000")
        motor.leftServo.duty(43); motor.rightServo.duty(109)
        utime.sleep(0.4)
    else:
        eyes.ultrasonicRGB1("00ff00","00ff00")
        motor.leftServo.duty(95); motor.rightServo.duty(43)
        utime.sleep(0.4)
    motor.leftServo.duty(0); motor.rightServo.duty(0)

print("Starting Tilt-Activated Rescue patrol")
while True:
    # Check tilt
    if tilt_pin.value() == 1:  # tilted
        print("Tilt detected! Entering Rescue Mode")
        # Flash blue eyes while waiting for button press
        while btn.value() == 1:  # button not pressed
            eyes.ultrasonicRGB1("0000ff","0000ff")
            utime.sleep(0.3)
            eyes.clearultrasonicRGB()
            utime.sleep(0.3)
        print("Button pressed! Stabilized, resuming patrol.")
        # small celebratory spin
        for _ in range(2):
            eyes.ultrasonicRGB1("00ff00","00ff00")
            motor.leftServo.duty(95); motor.rightServo.duty(95)
            utime.sleep(0.3)
            motor.leftServo.duty(0); motor.rightServo.duty(0)
            eyes.clearultrasonicRGB()
            utime.sleep(0.2)
    else:
        patrol_step()
    utime.sleep(0.1)
```

> **Tone:** Adventurous “rescue mission” with tilt and button.
> **Uses:** Motors + ultrasonic eyes + tilt sensor + push button (Sense + Interact).

---

## B. SENSE EXPANSION KIT (Microphone, Temperature & Humidity, Light Sensor, Tilt Sensor)

> **Note:** In combined examples, we avoid using the RGB ring or OLED (incompatible). We can combine with standard motors/ultrasonic eyes when appropriate, but in first 3 examples only Sense sensors.

### 1. Easy: “Noise Monitor” (Microphone)

**Goal:** Print sound level; if loud (>70), flash console message.

```python
from machine import ADC, Pin
from time import sleep

# Microphone on ADC pin 32
adc_mic = ADC(Pin(32))
adc_mic.width(ADC.WIDTH_12BIT); adc_mic.atten(ADC.ATTN_11DB)

def read_sound():
    val = adc_mic.read()  # 0–4095
    return (val * 100) // 4095

print("Starting Noise Monitor")
while True:
    lvl = read_sound()
    print("Sound level:", lvl)
    if lvl > 70:
        print("Whoa, that’s loud!")
    sleep(1)
```

> **Tone:** Casual “listening” test.

---

### 2. Medium: “Weather Mood” (Temperature & Humidity)

**Goal:** Read temp/humidity via DHT; light up eyes color via ultrasonic module’s LEDs (if available) or print mood.

```python
from ottosensors import DHT
from ottoneopixel import OttoUltrasonic
from time import sleep

d = DHT(26)  # DHT pin
eyes = OttoUltrasonic(18, 19)

print("Starting Weather Mood")
while True:
    temp = d.temperature()
    hum  = d.humidity()
    print(f"Temp: {temp}°C, Humidity: {hum}%")
    # Mood logic
    if temp < 18:
        color = "0000ff"  # blue = cold
    elif temp < 28:
        color = "00ff00"  # green = comfy
    else:
        color = "ff0000"  # red = hot
    eyes.ultrasonicRGB1(color, color)
    sleep(2)
```

> **Tone:** Playful “Otto shows mood based on weather.”

---

### 3. Intermediate: “Light & Tilt Alarm”

**Goal:** If environment too dark and robot is tilted, sound an alert (console or buzzer if available).

```python
from machine import ADC, Pin
from time import sleep
from ottosensors import DHT  # reuse if needed
from ottobuzzer import OttoBuzzer

# Light sensor on ADC pin 33
adc_light = ADC(Pin(33))
adc_light.width(ADC.WIDTH_12BIT); adc_light.atten(ADC.ATTN_11DB)
# Tilt sensor digital on pin 26
tilt_pin = Pin(26, Pin.IN)
# Buzzer for beep
buzzer = OttoBuzzer(25)

def read_light():
    val = adc_light.read()
    return (val * 100) // 4095

print("Starting Light & Tilt Alarm")
while True:
    light_lvl = read_light()
    tilted = (tilt_pin.value() == 1)
    print(f"Light: {light_lvl}, Tilted: {tilted}")
    if light_lvl < 30 and tilted:
        print("Alert! Dark & tilted!")
        buzzer.playEmoji("S_surprise")
    sleep(1)
```

> **Tone:** Detective “if it’s dark and shaken, alert!”

---

### 4. Harder: “Sound-Activated Thermostat” (Sense + Standard motors)

**Goal:** Use microphone to detect clap to toggle a “fan” (motor spin) when temperature > a threshold. No ring used.

```python
from machine import ADC, Pin
from ottosensors import DHT
from ottomotor import OttoMotor
from time import sleep

# Microphone
adc_mic = ADC(Pin(32))
adc_mic.width(ADC.WIDTH_12BIT); adc_mic.atten(ADC.ATTN_11DB)
# Temperature
d = DHT(26)
# Motor used as “fan” (e.g. an attached small fan on a servo pin)
fan = OttoMotor(13, 14)  # repurpose one side to drive fan; just demonstration
fan.leftServo.freq(50)

fan_on = False

def read_sound():
    return (adc_mic.read() * 100) // 4095

print("Starting Sound-Activated Thermostat")
while True:
    temp = d.temperature()
    print("Temp:", temp)
    lvl = read_sound()
    if lvl > 60 and temp > 25:
        # toggle fan
        fan_on = not fan_on
        if fan_on:
            print("Clap & hot → Fan ON")
            fan.leftServo.duty(95)
        else:
            print("Clap again → Fan OFF")
            fan.leftServo.duty(0)
        sleep(1)
    sleep(0.5)
```

> **Tone:** Practical “clap to toggle fan when warm,” combining Sense mic + DHT + motor.

---

### 5. Hard: “Tilt-Controlled Weather Reporter” (Sense + Interact + Standard eyes)

**Goal:** On startup, read temp/humidity; tilt forward to display temp via blinking eyes; tilt backward to display humidity via different blink pattern; press button (Interact) to repeat or exit.

```python
from machine import ADC, Pin
from ottosensors import DHT
from ottoneopixel import OttoUltrasonic
from time import sleep
from ottobuzzer import OttoBuzzer
# Interact button on pin 27
btn = Pin(27, Pin.IN, Pin.PULL_UP)
# Tilt sensor on pin 26
tilt_pin = Pin(26, Pin.IN)
# Temperature/Humidity
d = DHT(26)
# Eyes
eyes = OttoUltrasonic(18, 19)
# Optional buzzer for feedback
buzzer = OttoBuzzer(25)

def blink_eyes(color, times):
    for _ in range(times):
        eyes.ultrasonicRGB1(color, color)
        sleep(0.3)
        eyes.clearultrasonicRGB()
        sleep(0.2)

print("Tilt-Controlled Weather Reporter")
temp = d.temperature(); hum = d.humidity()
print(f"Readings: {temp}°C, {hum}%")
buzzer.playEmoji("S_happy")

while True:
    if tilt_pin.value() == 1:  # tilted forward
        print("Tilt forward: showing Temp")
        blink_eyes("ff0000", min(int(temp), 5))  # blink red up to 5 times
        sleep(1)
    elif tilt_pin.value() == 0:  # upright or backward? treat backward separately if possible
        # For demo: assume value==0 is backward
        print("Tilt backward: showing Humidity")
        blink_eyes("0000ff", min(int(hum//10), 5))  # blink blue, times proportional
        sleep(1)
    # Press button to re-read sensor
    if btn.value() == 0:
        print("Button pressed: updating readings")
        temp = d.temperature(); hum = d.humidity()
        buzzer.playEmoji("S_surprise")
        sleep(0.5)
    sleep(0.2)
```

> **Tone:** Interactive “tilt to see weather,” combining Sense sensors + Interact button + eyes.

---

## C. INTERACT EXPANSION KIT (Push Button, Potentiometer, Encoder with Button, Accelerometer)

> **Note:** In combined examples, avoid using ring if Sense modules also used simultaneously unless only Interact + Standard motors etc. For first 3 examples, only Interact modules.

### 1. Easy: “Button Blink”

**Goal:** Toggle built-in LED or eyes color when push button is pressed.

```python
from machine import Pin
from time import sleep
from ottoneopixel import OttoUltrasonic

btn = Pin(27, Pin.IN, Pin.PULL_UP)
eyes = OttoUltrasonic(18, 19)

print("Starting Button Blink test")
while True:
    if btn.value() == 0:  # pressed
        print("Button pressed → green eyes")
        eyes.ultrasonicRGB1("00ff00", "00ff00")
    else:
        eyes.clearultrasonicRGB()
    sleep(0.1)
```

> **Tone:** Simple “light up when pressed.”
> **Uses:** Push Button + eyes.

---

### 2. Medium: “Pot Volume Control”

**Goal:** Use potentiometer to adjust buzzer note frequency or LED brightness (with OttoNeoPixel ring if available alone). Here: adjust eye brightness color intensity.

```python
from machine import ADC, Pin
from time import sleep
from ottoneopixel import OttoUltrasonic

# Potentiometer on ADC pin 33
adc_pot = ADC(Pin(33))
adc_pot.width(ADC.WIDTH_12BIT); adc_pot.atten(ADC.ATTN_11DB)
eyes = OttoUltrasonic(18, 19)

print("Starting Pot Volume (eye brightness) control")
while True:
    val = adc_pot.read()  # 0–4095
    level = (val * 100) // 4095  # 0–100
    # Map to hex intensity 00–FF
    hexval = hex((level * 255) // 100)[2:].zfill(2)
    color = hexval * 3  # "RR"=hexval, so color string "RRGGBB"
    print(f"Pot level {level}% → setting eyes {color}")
    eyes.ultrasonicRGB1(color, color)
    sleep(0.2)
```

> **Tone:** “Twist to brighten eyes” metaphor.
> **Uses:** Potentiometer + eyes.

---

### 3. Intermediate: “Encoder Menu Selector”

**Goal:** Use rotary encoder to cycle through preset behaviors (e.g., wave, dance, beep) and press encoder button to execute. Requires OttoMotor and OttoBuzzer.

```python
from machine import Pin
from ottosensors import RotaryIRQ
from time import sleep
from ottomotor import OttoMotor
from ottobuzzer import OttoBuzzer

# Encoder pins CLK=17, DT=16, Button integrated or separate pin?
encoder = RotaryIRQ(pin_num_clk=17, pin_num_dt=16,
                    min_val=0, max_val=2, reverse=False,
                    range_mode=RotaryIRQ.RANGE_WRAP)
# Assume encoder button on pin 27
btn = Pin(27, Pin.IN, Pin.PULL_UP)

motor = OttoMotor(13, 14)
buzzer = OttoBuzzer(25)

options = ["wave", "dance", "beep"]
print("Encoder Menu: 0=wave,1=dance,2=beep")
encoder.set(value=0)

def execute_choice(idx):
    choice = options[idx]
    print(f"Executing {choice}")
    if choice == "wave":
        # hypothetical wave: spin one servo then other
        motor.leftServo.freq(50); motor.rightServo.freq(50)
        motor.leftServo.duty(95); motor.rightServo.duty(95)
        sleep(0.3)
        motor.leftServo.duty(0); motor.rightServo.duty(0)
    elif choice == "dance":
        buzzer.playEmoji("S_happy"); sleep(0.5)
        motor.leftServo.duty(95); motor.rightServo.duty(43)
        sleep(0.5); motor.leftServo.duty(0); motor.rightServo.duty(0)
    elif choice == "beep":
        buzzer.playNote(440, 200)
    sleep(0.2)

print("Rotate encoder to select, press to confirm.")
while True:
    val = encoder.value()
    print("Selection:", options[val])
    if btn.value() == 0:
        execute_choice(val)
        # debounce
        while btn.value() == 0:
            sleep(0.1)
    sleep(0.2)
```

> **Tone:** “Navigate menu with the knob, press to activate.”
> **Uses:** Encoder + button + motors + buzzer.

---

### 4. Harder: “Shake to Shuffle Lights” (Accelerometer + Motor)

**Goal:** If Otto is shaken (via accelerometer), randomly choose a motor routine or LED pattern. Assume accelerometer accessible via I2C (ADXL345); no ring to avoid kit conflict unless alone.

```python
from machine import Pin, SoftI2C
from adxl345 import ADXL345
from ottomotor import OttoMotor
from ottobuzzer import OttoBuzzer
from ottoneopixel import OttoUltrasonic
import random, utime

# Accelerometer on I2C pins
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
accel = ADXL345(i2c)
motor = OttoMotor(13, 14)
buzzer = OttoBuzzer(25)
eyes = OttoUltrasonic(18, 19)

motor.leftServo.freq(50); motor.rightServo.freq(50)

def detect_shake():
    x, y, z = accel.ReadXYZ()
    mag = abs(x) + abs(y) + abs(z)
    return mag > 30  # threshold

print("Shake to Shuffle!")
while True:
    if detect_shake():
        choice = random.choice(["spin", "blink", "beep"])
        print("Shaken! Doing:", choice)
        if choice == "spin":
            motor.leftServo.duty(95); motor.rightServo.duty(95)
            sleep(0.5); motor.leftServo.duty(0); motor.rightServo.duty(0)
        elif choice == "blink":
            for _ in range(3):
                eyes.ultrasonicRGB1("ff00ff","ff00ff")
                sleep(0.2)
                eyes.clearultrasonicRGB()
                sleep(0.2)
        else:  # beep
            buzzer.playEmoji("S_superHappy")
        # avoid rapid re-trigger
        utime.sleep(1)
    sleep(0.1)
```

> **Tone:** “Shake me for a surprise!” combining Interact accel + standard motors/eyes.
> **Uses:** Accelerometer + motors + eyes + buzzer.

---

### 5. Hard: “Pot-and-Encoder DJ” (Potentiometer + Encoder + Accelerometer + Buzzer)

**Goal:** Create a mini “DJ controller”: potentiometer adjusts tempo (BPM), encoder selects melody, shaking triggers random remix.

```python
from machine import ADC, Pin, SoftI2C
from ottosensors import RotaryIRQ
from adxl345 import ADXL345
from ottobuzzer import OttoBuzzer
from time import sleep, time
import random

# Potentiometer on ADC33 for tempo
adc_pot = ADC(Pin(33))
adc_pot.width(ADC.WIDTH_12BIT); adc_pot.atten(ADC.ATTN_11DB)
# Encoder for melody choice
encoder = RotaryIRQ(pin_num_clk=17, pin_num_dt=16,
                    min_val=0, max_val=2, reverse=False,
                    range_mode=RotaryIRQ.RANGE_WRAP)
# Accelerometer
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
accel = ADXL345(i2c)
buzzer = OttoBuzzer(25)

encoder.set(value=0)
melodies = [
    [262, 294, 330],   # C-D-E
    [330, 294, 262],   # E-D-C
    [262, 330, 262]    # C-E-C
]

def read_pot_tempo():
    val = adc_pot.read()  # 0–4095
    bpm = 60 + (val * 60) // 4095  # 60–120 BPM
    return bpm

def detect_shake():
    x,y,z = accel.ReadXYZ()
    return abs(x)+abs(y)+abs(z) > 35

print("DJ Mode: twist pot for tempo, turn knob for melody, shake for remix")
last_play = 0
while True:
    bpm = read_pot_tempo()
    melody = melodies[encoder.value()]
    interval = 60 / bpm
    now = time()
    if now - last_play >= interval:
        # Play next note in melody
        note = random.choice(melody) if detect_shake() else melody[now // int(interval) % len(melody)]
        print(f"Playing note {note} at {bpm} BPM")
        buzzer.playNote(note, int(interval*500))  # duration ~ half beat
        last_play = now
    sleep(0.05)
```

> **Tone:** “DJ with pot, knob, shake for remix.”
> **Uses:** Potentiometer + encoder + accelerometer + buzzer.

---

## D. EMOTE EXPANSION KIT (OLED Display, LED Matrix, MP3 Module)

> **Note:** OLED display is incompatible with ultrasonic eyes; LED Matrix can be used alone or with other Interact modules; MP3 module uses its own pins.

### 1. Easy: “Hello OLED”

**Goal:** Display “Hello Otto!” then clear.

```python
from machine import Pin, SoftI2C
from ssd1306 import SSD1306_I2C
from time import sleep

i2c = SoftI2C(scl=Pin(19), sda=Pin(18))
oled = SSD1306_I2C(128, 64, i2c)

print("Displaying greeting on OLED")
oled.fill(0)
oled.text("Hello Otto!", 0, 0, 1)
oled.show()
sleep(2)
oled.fill(0)
oled.show()
```

> **Tone:** Warm “greet yourself on OLED.”

---

### 2. Medium: “Matrix Emoji Blink”

**Goal:** Show a simple smile emoji on 8x8 LED matrix, blink it on/off.

```python
from neopixel import NeoPixel
from machine import Pin
from time import sleep

# 8x8 matrix on Pin 22 (64 LEDs)
matrix = NeoPixel(Pin(22), 64)

# Simple smile pattern bits (row-major, '1' means lit)
smile = [
    "00111100",
    "01000010",
    "10100101",
    "10000001",
    "10100101",
    "10011001",
    "01000010",
    "00111100"
]

def draw_pattern(pattern, r, g, b):
    for row in range(8):
        for col in range(8):
            idx = row*8 + col
            if pattern[row][col] == '1':
                matrix[idx] = (r, g, b)
            else:
                matrix[idx] = (0, 0, 0)
    matrix.write()

print("Blinking smile emoji on matrix")
while True:
    draw_pattern(smile, 0, 255, 0)  # green smile
    sleep(0.5)
    matrix.fill((0,0,0))
    matrix.write()
    sleep(0.5)
```

> **Tone:** Cheerful “blink your smile.”

---

### 3. Intermediate: “MP3 Jukebox”

**Goal:** Play a track from MP3 module when a (simulated) button press; here we just play next track in a loop.

```python
from ottobuzzer import Player  # MP3 player
from time import sleep

# DFPlayer on pins TX=16, RX=17
mp3 = Player(16, 17)
mp3.volume(20)  # set volume (0–30)
print("Starting Jukebox test")
for _ in range(3):
    print("Playing next track")
    mp3.play('next')  # play next track
    sleep(5)          # let it play for 5s
print("Jukebox test done")
```

> **Tone:** Casual “listen to 3 tracks.”

---

### 4. Harder: “OLED Status Dashboard” (OLED + Interact potentiometer + Standard sensor data)

**Goal:** Use potentiometer (Interact) to scroll through pages showing: distance (ultrasonic), temperature (if Sense present), or default message. Avoid ultrasonic eyes since OLED incompatible; but you can still read ultrasonic distance if eyes LEDs off. Here we only display numeric data on OLED.

```python
from machine import Pin, SoftI2C, ADC
from ssd1306 import SSD1306_I2C
from ottoneopixel import OttoUltrasonic  # for distance reading only; avoid eyes LEDs
from ottosensors import DHT
from time import sleep

# OLED
i2c = SoftI2C(scl=Pin(19), sda=Pin(18))
oled = SSD1306_I2C(128, 64, i2c)

# Potentiometer on ADC33
adc_pot = ADC(Pin(33))
adc_pot.width(ADC.WIDTH_12BIT); adc_pot.atten(ADC.ATTN_11DB)

# Ultrasonic distance (read only)
ultra = OttoUltrasonic(18, 19)
# Temperature (Sense)
d = DHT(26)

pages = ["Distance", "Temperature", "Hello"]
current = 0

def read_pot():
    val = adc_pot.read()
    # map 0–4095 to index 0–2
    return (val * len(pages)) // 4096

print("Starting OLED Dashboard (use pot to switch page)")
while True:
    sel = read_pot()
    if sel != current:
        current = sel
        oled.fill(0)
        oled.text(pages[current], 0, 0, 1)
        if pages[current] == "Distance":
            dist = int(ultra.readDistance())
            oled.text(f"{dist} cm", 0, 16, 1)
        elif pages[current] == "Temperature":
            t = d.temperature()
            h = d.humidity()
            oled.text(f"{t}C {h}%", 0, 16, 1)
        else:
            oled.text("Have fun!", 0, 16, 1)
        oled.show()
    sleep(0.2)
```

> **Tone:** Informative “dashboard you scroll with a knob.”
> **Uses:** OLED + potentiometer + ultrasonic reading + DHT.

---

### 5. Hard: “Matrix Music Visualizer” (MP3 Module + LED Matrix + Interact accelerometer)

**Goal:** Play an MP3 track and show simple animated bars on LED matrix based on accelerometer tilt as a proxy for “beat.” (Since real audio FFT is complex, we simulate visuals reacting to tilt: tilt left→left bars, tilt right→right bars.)

```python
from neopixel import NeoPixel
from machine import Pin, SoftI2C, ADC
from ottobuzzer import Player
from adxl345 import ADXL345
import time

# LED matrix 8x8 on pin 22
matrix = NeoPixel(Pin(22), 64)
# MP3
mp3 = Player(16, 17)
mp3.volume(20)
# Accelerometer on I2C
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))  # adjust pins if needed
accel = ADXL345(i2c)

def read_tilt():
    x, y, z = accel.ReadXYZ()
    return x  # use X-axis for left/right tilt

def draw_bars(tilt):
    # Map tilt (-max..+max) to bar height 0–8
    level = min(max((tilt + 50)//10, 0), 8)  # example mapping
    matrix.fill((0,0,0))
    # draw vertical bar on left or right
    if tilt < 0:
        # left side bar
        for i in range(level):
            matrix[i*8 + 0] = (0,0,255)  # first column
    else:
        # right side bar
        for i in range(level):
            matrix[i*8 + 7] = (0,255,0)  # last column
    matrix.write()

print("Starting Matrix Music Visualizer")
mp3.play()  # start playing track
time.sleep(1)  # let it start
start = time.time()
while time.time() - start < 20:  # run 20 seconds
    tilt = read_tilt()
    print("Tilt X:", tilt)
    draw_bars(tilt)
    time.sleep(0.1)
matrix.fill((0,0,0)); matrix.write()
print("Visualizer done")
```

> **Tone:** Creative “visualizer reacts to tilt as if beat-driven.”
> **Uses:** LED Matrix + MP3 module + accelerometer.

---

**Let me know** if this style and these examples fit your needs, or if you’d like adjustments (e.g., different sensor pins or more playful narratives). Once approved, you can adapt these patterns to your exact pin mappings and hardware details. Enjoy experimenting!
