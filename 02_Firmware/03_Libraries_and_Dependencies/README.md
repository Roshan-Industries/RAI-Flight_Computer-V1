# FC_V1 Libraries and Dependencies

This document records the principal software libraries and development
dependencies required to compile and operate the FC_V1 firmware.

## 1. Development Environment

- Platform: ESP32
- Microcontroller board: ESP32 Development Board / ESP-WROOM-32
- Development environment: Arduino IDE
- Programming language: C/C++
- Communication interface: USB Serial

## 2. Firmware Dependencies

The FC_V1 firmware uses libraries supporting sensor acquisition, display
operation, I²C communication and recovery-servo control.

| Library / Dependency | Application in FC_V1 |
|---|---|
| Wire | I²C communication between ESP32 and connected peripherals |
| Adafruit BMP280 Library | BMP280 barometric-pressure and relative-altitude measurement |
| Adafruit Unified Sensor | Supporting sensor interface dependency |
| Adafruit GFX Library | Graphics support for the OLED display |
| Adafruit SSD1306 | SSD1306 OLED display control |
| MPU6050 Library | MPU6050 acceleration and angular-rate acquisition |
| ESP32Servo | SG90 recovery-servo control |

## 3. Hardware Interfaces

The principal firmware-controlled interfaces are:

- I²C SDA — GPIO 21
- I²C SCL — GPIO 22
- Recovery servo signal — GPIO 18
- Buzzer — GPIO 25

Additional GPIO assignments for the push button and status LEDs are defined
within the final firmware source code.

## 4. Dependency Management

FC_V1 was developed as an experimental engineering prototype using standard
Arduino/ESP32 libraries.

External library source files are not duplicated within this repository.
Required libraries should be installed through the Arduino IDE Library Manager
or their respective official distribution sources.

Where exact library versions are known, they should be recorded to improve
build reproducibility.

## 5. Reproducibility Note

Future FC_V2 development should freeze and document exact library versions,
ESP32 board-package versions and build configuration to provide a fully
reproducible firmware environment.

The FC_V1 dependency record represents the development environment used for
prototype implementation and bench verification.
