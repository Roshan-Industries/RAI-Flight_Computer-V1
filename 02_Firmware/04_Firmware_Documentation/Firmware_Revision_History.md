# FC_V1 Firmware Revision History

## Purpose

This document records the development history of the FC_V1 firmware from
individual subsystem testing to the final integrated bench-verified version.

## Development Progression

FC_V1 firmware development followed an incremental integration approach:

Component Testing
→ Sensor Integration
→ User Interface Integration
→ Flight-State Logic
→ Recovery Control
→ Landing Detection
→ User-Control Integration
→ Final Bench Verification

## Revision Record

| Revision | Development Stage | Status |
|---|---|---|
| Early Development | ESP32 basic execution and individual component testing | Superseded |
| Sensor Integration | BMP280 and MPU6050 acquisition integrated | Superseded |
| Display Integration | OLED diagnostic/state display incorporated | Superseded |
| State-Machine Development | IDLE, ARMED, ASCENT, APOGEE, DESCENT and LANDED logic implemented | Superseded |
| Recovery Integration | Servo recovery-control output incorporated | Superseded |
| Landing/Beacon Integration | Landing confirmation and post-landing indication incorporated | Superseded |
| V1.6.5 | Push-button/user-control integration and final integrated configuration | FINAL / FROZEN |

## Final Firmware

Final FC_V1 firmware:

`FC_V1_v1.6.5_ButtonIntegrated.ino`

Status:

**FINAL — BENCH VERIFIED — FROZEN**

## Final Verification

The final hardware and firmware configuration successfully completed five
repeated integrated bench-test sequences.

The verified nominal sequence was:

IDLE → ARMED → ASCENT → APOGEE → DESCENT → LANDED

The tests demonstrated repeatable functional operation under the controlled
bench-test conditions used during FC_V1 development.

## Configuration Control

FC_V1 V1.6.5 represents the frozen firmware baseline associated with the
completed FC_V1 prototype and technical report.

Future improvements should not overwrite this baseline.

Substantial changes involving PCB architecture, data logging, sensor
processing, power architecture or flight-oriented verification should proceed
under the FC_V2 development programme.

## Verification Classification

**Integrated Functional Prototype — Bench Verified**

FC_V1 has not been demonstrated as flight-qualified hardware.
