# FC_V1 Firmware Pin Mapping

## Purpose

This document records the ESP32 GPIO assignments used by the FC_V1 firmware
and integrated hardware prototype.

## Principal Pin Assignments

| Function | ESP32 GPIO | Interface / Device |
|---|---:|---|
| I²C SDA | GPIO 21 | BMP280, MPU6050 and SSD1306 OLED |
| I²C SCL | GPIO 22 | BMP280, MPU6050 and SSD1306 OLED |
| Recovery Servo Signal | GPIO 18 | SG90 servo |
| Recovery Buzzer | GPIO 25 | Audible recovery indication |

## I²C Architecture

The BMP280, MPU6050 and SSD1306 OLED share the ESP32 I²C communication bus.

ESP32
├── GPIO 21 → SDA
└── GPIO 22 → SCL

Shared I²C Bus
├── BMP280
├── MPU6050
└── SSD1306 OLED

Each peripheral is addressed independently over the shared bus.

## Recovery-Control Interface

The recovery actuator is controlled through the servo signal output:

GPIO 18 → Servo Control Signal → SG90 Servo

The servo represents the physical recovery-actuation output of FC_V1.

## Recovery-Beacon Interface

GPIO 25 is assigned to the buzzer used for audible post-landing recovery
indication.

## Additional Digital I/O

FC_V1 also incorporates:

- user push-button input;
- status LEDs;
- visual state indication.

Their exact GPIO assignments should be taken directly from the final firmware
source before being added to this document.

## Configuration Status

This pin mapping corresponds to the final FC_V1 integrated prototype.

Any GPIO changes introduced during FC_V2 development should be documented
separately and should not modify this frozen FC_V1 record.
