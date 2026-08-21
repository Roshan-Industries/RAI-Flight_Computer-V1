# FC_V1 Hardware Pinout

## ESP32 Interface Allocation

| ESP32 Pin | Connected Hardware | Function |
|---|---|---|
| GPIO 21 | BMP280 + MPU6050 + OLED | I²C SDA |
| GPIO 22 | BMP280 + MPU6050 + OLED | I²C SCL |
| GPIO 18 | SG90 Servo | Recovery actuator signal |
| GPIO 25 | Buzzer | Audible indication |
| GPIO 26 | Green LED | Status indication |
| GPIO 27 | Yellow LED | Status indication |
| GPIO 32 | Blue LED | Flight-state indication |
| GPIO 33 | Red LED | Recovery/warning indication |
| GPIO 14 | Push Button | ARM / DISARM / RESET input |
| 3.3 V | Sensor/display supply | Logic power |
| GND | All subsystems | Common electrical reference |

## I²C Devices

| Device | Interface |
|---|---|
| BMP280 | Shared I²C bus |
| MPU6050 | Shared I²C bus |
| SSD1306 OLED | Shared I²C bus |

## Servo Interface

The SG90 servo uses:

- Signal → GPIO 18
- Power → appropriate servo supply
- Ground → common system ground

The servo must not be powered from the ESP32 3.3 V output.

## Push-Button Interface

The user push button is connected between:

`GPIO 14 → Push Button → GND`

The firmware configures GPIO 14 using the ESP32 internal pull-up resistor.

## Status

This pinout corresponds to the final FC_V1 integrated prototype and frozen
V1.6.5 firmware configuration.
