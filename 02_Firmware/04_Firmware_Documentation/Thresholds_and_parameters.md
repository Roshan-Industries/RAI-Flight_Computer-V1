# FC_V1 Firmware Thresholds and Parameters

## Purpose

This document records the principal configurable parameters used by the final
FC_V1 flight-event detection and recovery-control firmware.

Values in this document must correspond to the frozen final FC_V1 firmware.

## Launch Detection

| Parameter | Value | Purpose |
|---|---:|---|
| Acceleration threshold | [FROM FINAL CODE] | Primary simulated launch-event criterion |
| Altitude confirmation threshold | [FROM FINAL CODE] | Additional launch confirmation |
| Confirmation duration/count | [FROM FINAL CODE] | Rejects transient measurements |

## Apogee Detection

| Parameter | Value | Purpose |
|---|---:|---|
| Peak-altitude decrease threshold | [FROM FINAL CODE] | Detects decrease from recorded peak altitude |
| Confirmation condition | [FROM FINAL CODE] | Confirms apogee before recovery actuation |

## Landing Detection

| Parameter | Value | Purpose |
|---|---:|---|
| Altitude-stability threshold | [FROM FINAL CODE] | Evaluates approximately stationary altitude |
| Acceleration/stability threshold | [FROM FINAL CODE] | Evaluates stationary condition |
| Confirmation duration | [FROM FINAL CODE] | Prevents premature landing declaration |

## Recovery Servo

| Parameter | Value | Purpose |
|---|---:|---|
| Servo GPIO | 18 | Recovery actuator control |
| STOWED position | [FROM FINAL CODE] | Normal/pre-recovery position |
| DEPLOYED position | [FROM FINAL CODE] | Recovery-command position |

## Recovery Beacon

| Parameter | Value | Purpose |
|---|---:|---|
| Buzzer GPIO | 25 | Audible recovery indication |
| Activation state | LANDED | Post-landing recovery indication |
| Timing behaviour | [FROM FINAL CODE] | Buzzer operating sequence |

## Sensor Acquisition

| Parameter | Value |
|---|---:|
| I²C SDA | GPIO 21 |
| I²C SCL | GPIO 22 |
| BMP280 configuration | [FROM FINAL CODE] |
| MPU6050 configuration | [FROM FINAL CODE] |
| Sensor update interval | [FROM FINAL CODE] |

## Engineering Note

The FC_V1 thresholds were developed for functional prototype and controlled
bench verification.

They must not be interpreted as validated flight parameters for an actual
launch vehicle.

Future flight-oriented parameters should be established through sensor
characterization, recorded test data and representative ground/flight testing.
