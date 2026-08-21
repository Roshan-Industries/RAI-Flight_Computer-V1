# FC_V1 — ESP32-Based Flight Computer

**Roshan Aerospace Industries (RAI)**  
**Project:** Flight Computer V1  
**Document:** RAI-FC-2026-001  
**Firmware:** FC_V1_v1.6.5_ButtonIntegrated  
**Status:** Integrated Functional Prototype — Bench Verified

---

## Overview

FC_V1 is an experimental ESP32-based flight-computer prototype developed to
demonstrate the integration of sensing, embedded flight-state logic,
recovery-control actuation, user interaction, and post-landing indication
within a single functional system.

The project was developed incrementally from individual component testing
through breadboard integration, perfboard construction, firmware development,
and repeated integrated bench verification.

FC_V1 is a proof-of-function engineering prototype and is **not
flight-qualified avionics**.

---

## Final Prototype

![FC_V1 Final Prototype](05_Project_Images/03_Final_Prototype/FC_V1_Final_Prototype_001_Front.jpg)

*Completed FC_V1 perfboard prototype following hardware integration, firmware
implementation, and bench verification.*

---

## System Architecture

FC_V1 integrates:

- ESP32 processing platform
- BMP280 barometric-pressure sensor
- MPU6050 inertial measurement unit
- SSD1306 OLED diagnostic display
- SG90 recovery-actuation servo
- user ARM / DISARM / RESET push button
- four visual status indicators
- audible post-landing recovery beacon
- perfboard-based point-to-point hardware integration

The principal functional chain is:

`Physical Measurement → Sensor Acquisition → Embedded Processing → Flight-State Evaluation → Event Detection → Recovery Command → Physical Actuation → Post-Landing Indication`

---

## Flight-State Machine

The nominal flight-event sequence is:

`IDLE → ARMED → ASCENT → APOGEE → DESCENT → LANDED`

The firmware uses sensor measurements together with confirmation logic to
evaluate state transitions rather than relying on individual instantaneous
measurements.

---

## Hardware

| Component | Function |
|---|---|
| ESP32 | Main processing and firmware execution |
| BMP280 | Barometric pressure and relative-altitude measurement |
| MPU6050 | Inertial acceleration and angular-rate measurement |
| SSD1306 OLED | Local diagnostic and flight-state display |
| SG90 Servo | Recovery-actuation demonstration |
| Push Button | ARM / DISARM / RESET user input |
| Status LEDs | Visual system-state indication |
| Buzzer | Post-landing audible recovery indication |
| Perfboard | Integrated prototype construction platform |

---

## Principal Interfaces

| Function | ESP32 GPIO |
|---|---:|
| I²C SDA | GPIO 21 |
| I²C SCL | GPIO 22 |
| Recovery Servo | GPIO 18 |
| Buzzer | GPIO 25 |
| Push Button | GPIO 14 |

Complete interface information is available in:

`03_Hardware/02_Pinout/FC_V1_Pinout.md`

---

## Firmware

The final frozen FC_V1 firmware is:

`02_Firmware/01_Final/FC_V1_v1.6.5_ButtonIntegrated.ino`

Firmware development included:

- individual sensor testing
- combined I²C sensor acquisition
- OLED diagnostic interface
- flight-state machine
- launch-event detection
- peak-altitude tracking
- apogee detection
- recovery actuation
- landing detection
- LED status indication
- recovery beacon
- physical user-control integration

Detailed firmware documentation is available under:

`02_Firmware/04_Firmware_Documentation/`

---

## Verification

The completed FC_V1 hardware and final firmware configuration were subjected
to repeated integrated simulated flight-event bench testing.

### Final Integrated Test Series

| Test | Result |
|---|:---:|
| Test 1 | PASS |
| Test 2 | PASS |
| Test 3 | PASS |
| Test 4 | PASS |
| Test 5 | PASS |

**Final result: 5/5 integrated bench-test sequences passed.**

The tests exercised the implemented sequence from user arming through simulated
launch, ascent, apogee detection, recovery actuation, descent, landing
detection, and post-landing recovery indication.

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
- representative launch vibration
- shock qualification
- thermal qualification
- electromagnetic compatibility
- flight reliability
- flight qualification

Accordingly, FC_V1 is classified as:

> **Integrated Functional Prototype — Bench Verified**

---

## Repository Structure

```text
RAI-FC-V1/
│
├── 01_Report/
│   └── Technical report and source documentation
│
├── 02_Firmware/
│   ├── Final firmware
│   ├── Development firmware
│   ├── Libraries and dependencies
│   └── Firmware documentation
│
├── 03_Hardware/
│   ├── Wiring diagrams
│   ├── Pinout
│   ├── Bill of materials
│   └── Hardware architecture
│
├── 04_Testing/
│   └── Test procedures, results and evidence
│
├── 05_Project_Images/
│   ├── Breadboard prototype
│   ├── Perfboard construction
│   ├── Final prototype
│   └── Testing
│
├── 06_Documentation/
├── 07_Project_Media/
├── 08_Archive/
│
├── README.md
├── LICENSE
└── CITATION.cff
```

---

## Technical Report

The complete FC_V1 engineering report is available under:

`01_Report/`

**Document ID:** RAI-FC-2026-001

The report documents:

- system requirements
- hardware architecture
- firmware architecture
- flight-state logic
- prototype development
- bench verification
- engineering limitations
- FC_V2 development roadmap
- hardware configuration
- bill of materials
- firmware architecture
- detailed test records

---

## Development Progression

FC_V1 was developed using an incremental engineering workflow:

`Component Testing`
→ `Breadboard Integration`
→ `Combined Subsystem Testing`
→ `Flight-State Logic`
→ `Perfboard Integration`
→ `Firmware Refinement`
→ `Integrated Bench Testing`
→ `5/5 Final Test Pass`

This approach allowed subsystem-level faults to be identified before complete
system integration.

---

## Limitations

FC_V1 uses development-oriented hardware including an ESP32 development board,
commercial sensor modules, removable headers, and manually implemented
point-to-point perfboard wiring.

The prototype does not incorporate:

- dedicated flight PCB
- onboard persistent flight-data logging
- redundant sensing
- dedicated flight-power architecture
- protective flight enclosure
- environmental qualification

These limitations define the principal engineering objectives for FC_V2.

---

## FC_V2 Direction

Future development is intended to investigate:

- purpose-designed PCB architecture
- onboard flight-data logging
- improved sensor processing and filtering
- improved inertial and barometric sensing
- dedicated power architecture
- mechanically secured interfaces
- protective enclosure
- integrated recovery-system ground testing
- vibration and environmental evaluation
- controlled flight testing

FC_V1 remains frozen as the first completed bench-verified baseline of the RAI
flight-computer development programme.

---

## Safety and Engineering Status

FC_V1 is an experimental educational and engineering-development prototype.

The project has not undergone certification, environmental qualification, or
flight qualification and should not be represented as certified or
flight-qualified avionics.

---

## License

Software/source code in this repository is licensed under the Apache License
2.0 unless otherwise stated.

Engineering reports, photographs, diagrams, RAI branding, logos, and other
non-software materials are not automatically licensed under Apache-2.0.
See `LICENSE` for the complete licensing terms and scope.

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

**FC_V1 — Flight Computer V1**  
2026
