# RAI Flight Computer V1 (FC_V1)

**Roshan Aerospace Industries (RAI)**  
**Project:** Flight Computer V1  
**Document ID:** RAI-FC-2026-001  
**Firmware Baseline:** FC_V1_v1.6.5_ButtonIntegrated  
**Status:** Integrated Functional Prototype — Bench Verified  
**Year:** 2026

---

## Overview

FC_V1 is an experimental ESP32-based flight-computer prototype developed to
demonstrate the integration of sensing, embedded flight-state logic,
recovery-control actuation, user interaction, and post-landing indication
within a single functional embedded system.

Development progressed incrementally from individual component verification
through breadboard integration, combined subsystem testing, perfboard
construction, firmware development, and repeated integrated bench testing.

The completed prototype successfully executed the implemented flight-event
sequence during five repeated simulated bench-test runs.

> **Engineering Status:** FC_V1 is a proof-of-function engineering prototype.
> It is not certified or flight-qualified avionics.

---

## Final Prototype

![FC_V1 Final Prototype](04_Project_Images/03_Final_Prototype/FC_V1_Final_Prototype_001_Front.jpg)

*Completed FC_V1 integrated perfboard prototype following hardware integration,
firmware implementation, and bench verification.*

---

## System Architecture

FC_V1 integrates:

- ESP32 processing platform
- BMP280 barometric-pressure sensor
- MPU6050 inertial measurement unit
- SSD1306 OLED diagnostic display
- SG90 recovery-actuation servo
- ARM / DISARM / RESET push-button interface
- four visual status indicators
- audible post-landing recovery beacon
- perfboard-based point-to-point hardware integration

The principal functional chain is:

`Physical Measurement → Sensor Acquisition → Embedded Processing → Flight-State Evaluation → Event Detection → Recovery Command → Physical Actuation → Post-Landing Indication`

The BMP280, MPU6050, and OLED share the ESP32 I²C communication bus.

---

## Flight-State Machine

The implemented nominal state progression is:

`IDLE → ARMED → ASCENT → APOGEE → DESCENT → LANDED`

Each transition represents a defined change in system behaviour.

The firmware evaluates sensor measurements and confirmation conditions before
allowing relevant state transitions rather than controlling the recovery
sequence from a single instantaneous sensor measurement.

### State Functions

| State | Principal Function |
|---|---|
| IDLE | System initialized; flight-event transitions inhibited |
| ARMED | System prepared for launch-event detection |
| ASCENT | Launch confirmed; altitude monitored and peak altitude tracked |
| APOGEE | Apogee criteria confirmed; recovery command initiated |
| DESCENT | Recovery output active; landing conditions monitored |
| LANDED | Landing confirmed; post-landing indications activated |

---

## Hardware

| Component | Function |
|---|---|
| ESP32 | Main processing and firmware execution |
| BMP280 | Barometric-pressure and relative-altitude measurement |
| MPU6050 | Acceleration and angular-rate measurement |
| SSD1306 OLED | Local diagnostic and flight-state display |
| SG90 Servo | Recovery-actuation demonstration |
| Push Button | ARM / DISARM / RESET user input |
| Status LEDs | Visual system-state indication |
| Buzzer | Post-landing audible recovery indication |
| Perfboard | Mechanical and electrical integration platform |

---

## Electrical Interfaces

### I²C Bus

The BMP280, MPU6050, and SSD1306 OLED share:

| Function | ESP32 GPIO |
|---|---:|
| I²C SDA | GPIO 21 |
| I²C SCL | GPIO 22 |

### Control and Status Interfaces

| Function | ESP32 GPIO |
|---|---:|
| Recovery Servo Signal | GPIO 18 |
| Buzzer | GPIO 25 |
| Green Status LED | GPIO 26 |
| Yellow Status LED | GPIO 27 |
| Blue Status LED | GPIO 32 |
| Red Status LED | GPIO 33 |
| ARM / DISARM / RESET Button | GPIO 14 |

### Power Architecture

- BMP280, MPU6050 and OLED: **3.3 V**
- Logic and sensor grounds: **common ground**
- Servo: **separate appropriate supply rail**
- Servo ground: **common with system ground**

Complete interface information is available under:

`03_Hardware/02_Pinout/`

---

## Firmware

The frozen FC_V1 firmware baseline is located at:

`02_Firmware/01_Final/FC_V1_v1.6.5_ButtonIntegrated.ino`

Firmware development included:

- individual sensor verification
- combined I²C sensor acquisition
- OLED diagnostic interface
- flight-state-machine implementation
- launch-event detection
- peak-altitude tracking
- apogee detection
- recovery-command generation
- servo actuation
- landing detection
- LED status indication
- post-landing recovery beacon
- physical user-control integration

Detailed firmware documentation is available under:

`02_Firmware/04_Firmware_Documentation/`

Development firmware is retained separately under:

`02_Firmware/02_Development/`

---

## Verification

The completed FC_V1 hardware and frozen firmware configuration were subjected
to repeated integrated simulated flight-event bench testing.

### Final Integrated Test Series

| Test | Result |
|---|:---:|
| Test 1 | PASS |
| Test 2 | PASS |
| Test 3 | PASS |
| Test 4 | PASS |
| Test 5 | PASS |

**Result: 5/5 integrated bench-test sequences passed under the defined test conditions.**

The test sequence exercised:

`IDLE → ARMED → ASCENT → APOGEE → DESCENT → LANDED`

The repeated tests verified successful operation of:

- system initialization
- user-controlled arming
- simulated launch detection
- transition into ASCENT
- peak-altitude tracking
- simulated apogee detection
- recovery-command generation
- servo actuation
- transition into DESCENT
- simulated landing detection
- transition into LANDED
- post-landing visual and audible indication

No unintended state transition or failure to complete the nominal sequence was
observed during the five recorded integrated tests.

---

## Verification Boundary

### Demonstrated

- individual sensor operation
- integrated sensor acquisition
- flight-state-machine execution
- simulated launch-event detection
- peak-altitude tracking
- simulated apogee detection
- recovery-command generation
- servo actuation
- simulated landing detection
- post-landing visual and audible indication
- repeated integrated bench operation

### Not Demonstrated

- actual rocket flight
- actual parachute deployment
- representative aerodynamic loading
- representative launch vibration
- shock qualification
- thermal/environmental qualification
- electromagnetic compatibility
- statistical flight reliability
- flight qualification

The five bench tests demonstrate repeatability under the specific controlled
test conditions employed. They should not be interpreted as statistical
reliability data or evidence of flight qualification.

Accordingly, FC_V1 is classified as:

> **Integrated Functional Prototype — Bench Verified**

---

## Repository Structure

```text
RAI-FC-V1/
│
├── 01_Report/
│   ├── Source/
│   └── Report_Flight_Computer_V1.1.pdf
│
├── 02_Firmware/
│   ├── 01_Final/
│   ├── 02_Development/
│   ├── 03_Libraries_and_Dependencies/
│   ├── 04_Firmware_Documentation/
│   └── README.md
│
├── 03_Hardware/
│   ├── 01_Wiring_Diagrams/
│   ├── 02_Pinout/
│   ├── 03_BOM/
│   ├── 04_Architecture/
│   └── README.md
│
├── 04_Project_Images/
│   ├── 01_Breadboard_Prototype/
│   ├── 02_Perfboard_Construction/
│   ├── 03_Final_Prototype/
│   └── 04_Testing/
│
├── CITATION.cff
├── LICENSE
├── NOTICE
└── README.md
```

---

## Technical Report

The complete FC_V1 engineering report is available at:

`01_Report/Report_Flight_Computer_V1.1.pdf`

**Document ID:** RAI-FC-2026-001

The report documents:

- project objectives and requirements
- system architecture
- hardware architecture
- firmware architecture
- flight-state logic
- event-detection methodology
- prototype development
- integrated bench verification
- engineering limitations
- development evidence
- bill of materials
- hardware configuration
- detailed test records
- FC_V2 development direction

---

## Development Progression

FC_V1 was developed using an incremental engineering workflow:

`Component Preparation`
→ `Individual Module Testing`
→ `Breadboard Integration`
→ `Combined Subsystem Verification`
→ `Flight-State Logic Development`
→ `Perfboard Construction`
→ `Point-to-Point Wiring`
→ `Integrated Firmware Testing`
→ `Repeated Bench Verification`

This approach allowed hardware, wiring, sensor, and firmware faults to be
identified at subsystem level before final integration.

---

## Known Limitations

FC_V1 uses development-oriented hardware including an ESP32 development board,
commercial sensor modules, removable headers, and manually implemented
point-to-point perfboard wiring.

The prototype does not incorporate:

- purpose-designed flight PCB
- onboard persistent flight-data logging
- redundant sensing
- dedicated flight-power architecture
- mechanically locked flight connectors
- protective flight enclosure
- environmental qualification
- flight qualification

These limitations are intentional characteristics of the FC_V1 development
stage and define engineering objectives for subsequent hardware iterations.

---

## FC_V2 Development Direction

FC_V2 is intended to build upon the architecture demonstrated by FC_V1 rather
than replace it without evidence.

Future development is intended to investigate:

- purpose-designed PCB architecture
- onboard flight-data logging
- improved sensor processing and filtering
- improved inertial and barometric sensing
- dedicated power architecture
- mechanically secured interfaces
- protective enclosure
- improved recovery-system integration
- vibration and environmental evaluation
- controlled flight testing

The development philosophy remains:

`Design → Implement → Test → Record → Analyse → Improve`

FC_V1 therefore remains frozen as the first completed and experimentally
bench-verified hardware baseline in the RAI flight-computer development
programme.

---

## Safety and Engineering Status

FC_V1 is an experimental educational and engineering-development prototype.

It has not undergone certification, environmental qualification, flight
qualification, or representative operational rocket-flight testing.

Nothing in this repository should be interpreted as certification of the
hardware for operational flight use.

---

## License

Software and source code in this repository are licensed under the
**Apache License 2.0**, unless otherwise stated.

Engineering reports, photographs, diagrams, RAI branding, logos, and other
non-software materials are not automatically licensed under Apache-2.0.

See `LICENSE` and `NOTICE` for the applicable terms and notices.

---

## Citation

Citation metadata is provided in:

`CITATION.cff`

If this project contributes to academic, educational, or engineering work,
please cite the repository using the provided citation metadata.

---

## Author

**Rajnish Roshan**  
Roshan Aerospace Industries (RAI)

**RAI Flight Computer V1 — FC_V1**  
**RAI-FC-2026-001**  
2026
