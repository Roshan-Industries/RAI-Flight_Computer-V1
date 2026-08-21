# FC_V1 Hardware

This directory contains hardware documentation for the Roshan Aerospace
Industries Flight Computer V1 (FC_V1) integrated functional prototype.

## Contents

### 01_Wiring_Diagrams
Detailed electrical interconnections between the ESP32, sensors, display,
status indicators, user controls and recovery-actuation hardware.

### 02_Pinout
Physical ESP32 GPIO and peripheral interface assignments.

### 03_BOM
Prototype-level bill of materials for the completed FC_V1 assembly.

### 04_Architecture
High-level hardware architecture and subsystem relationships.

## Hardware Configuration

FC_V1 integrates:

- ESP32 processing platform
- BMP280 barometric sensor
- MPU6050 inertial measurement unit
- SSD1306 OLED display
- SG90 recovery-actuation servo
- four status LEDs
- audible recovery buzzer
- ARM/DISARM/RESET push button
- perfboard-based point-to-point electrical integration

## Configuration Status

Hardware Revision: FC_V1

Firmware Baseline:
`FC_V1_v1.6.5_ButtonIntegrated.ino`

Status:

**Integrated Functional Prototype — Bench Verified**

FC_V1 is an experimental prototype and is not flight-qualified hardware.
