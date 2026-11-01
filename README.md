# MarioKart
Firmware and HTML control page for UCD Elecsoc's MarioKart Makerthon.

## Overview
I designed the circuit and firmware for UCD Elecsoc's makerthon event that contestants will use/build upon. The mechanical design was done by my teammate.

## Features
- **Bluetooth and WiFi control**: connect over Bluetooth Low Energy from a laptop, or join the car's own WiFi network and use any device (phone, tablet, laptop). The car serves the control page itself.
- **Three input modes**: on-screen joysticks, keyboard, or a PS4/PS5/Xbox controller, with remappable keys and buttons.
- **Custom drive equations**: edit the left and right motor equations (e.g. `(W - S) - D`) live in the browser to change how the car steers.
- **Weapon servo**: hold L or R to sweep the arm, with adjustable swing and retract speeds.
- **Hardware self-test**: press the BOOT button or click "Run hardware test" to check both motors and the servo without any other software.
- **Safety watchdog**: the motors stop if the car hears nothing for 400 ms, or if the connection drops.
- **Live diagnostics**: motor speeds, servo angle and command count reported back to the page.

## Hardware
- ESP32-C6 DevKit
- L298N motor driver
- 2 × DC motors
- 2 × 18650 Li-ion cells with a BMS
- SG90 micro servo

![Hardware Overview](buildguideoverview.png)

## How it Works
The ESP32 talks to the car either via Bluetooth or WiFi on a web interface, using the NimBLE-Arduino and WiFi libraries in the Arduino IDE.

The motor controls are connected to the L298N via the GPIO pins from the ESP32, along with the servo. The 18650s power the DC motors directly, while for the ESP32 and the servo the voltage is stepped down to 5V. The BMS provides protection for the Li-ion cells.

## Wiring
| Connection | |
|---|---|
| Cells | left + down (B+), right + up |
| BMS red → switch → L298N 12V | Power in |
| BMS black → L298N GND | Ground |
| L298N 5V → breadboard + row | 5V rail |
| L298N GND → breadboard − row | Ground rail |
| OUT1 / OUT2 → left motor | |
| OUT3 / OUT4 → right motor | |
| 5V-EN cap ON, ENA / ENB caps OFF | L298N jumpers |
| ESP32 5V → + row, ESP32 GND → − row | ESP32 power |
| Servo red → + row, brown → − row | Servo power |

| L298N / servo pin | ESP32 GPIO |
|---|---|
| ENA | 6 |
| IN1 | 7 |
| IN2 | 10 |
| ENB | 20 |
| IN3 | 22 |
| IN4 | 21 |
| Servo signal (orange) | 18 |

Nothing should bridge the + and − rails.

## Getting Started
1. **Install the board**: Arduino IDE → Boards Manager → "esp32" by Espressif (3.x). Select **ESP32C6 Dev Module**.
2. **Set the partition**: Tools → Partition Scheme → **Huge APP (3MB No OTA/1MB SPIFFS)**.
3. **Add the library**: Library Manager → **NimBLE-Arduino** by h2zero, version 2.x.
4. **Name your car**: open [`firmware/mariokartfinal/mariokartfinal.ino`](firmware/mariokartfinal/mariokartfinal.ino) (keep `page.h` in the same folder). Change `DEVICE_NAME` (and ideally `WIFI_PASSWORD`) so every car is unique.
5. **Upload**: unplug the ESP32 5V wire, connect USB to the UART port and click Upload.
6. **Check it**: open the Serial Monitor at 115200 baud. You should see three `[PWM] ... OK` lines, then `[READY]`.

## Using the Controls
- **WiFi (any device)**: join the car's WiFi network (named after `DEVICE_NAME`), then open `http://192.168.4.1`.
- **Bluetooth (laptop)**: open [`controller/mariokartfinal.html`](controller/mariokartfinal.html) in Chrome, Edge or Opera and click *Connect via Bluetooth*.

## Key Challenges
The main challenge was stepping down the voltage to power the servo without making the circuit design more complicated. I initially wanted to implement a buck converter, however that would require the contestants to solder, which raised safety concerns within my team, so we scrapped the idea and went with stepping down the voltage through the L298N.

## Licence
MIT. See [LICENSE](LICENSE).
