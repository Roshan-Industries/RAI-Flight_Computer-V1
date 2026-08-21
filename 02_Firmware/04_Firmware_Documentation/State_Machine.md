# FC_V1 Flight-State Machine

## Purpose

FC_V1 uses a deterministic state-machine architecture to evaluate simulated
flight events and coordinate recovery-related outputs.

## Nominal State Sequence

IDLE → ARMED → ASCENT → APOGEE → DESCENT → LANDED

## State Definitions

### IDLE

Initial operating state after system startup.

Principal behaviour:

- initializes the system;
- acquires sensor measurements;
- provides diagnostic information;
- inhibits flight-event progression until user arming.

### ARMED

The system is prepared for launch-event detection.

Principal behaviour:

- continues sensor acquisition;
- monitors launch-detection conditions;
- maintains recovery output in the stowed condition.

### ASCENT

Entered after confirmation of the simulated launch event.

Principal behaviour:

- continuously acquires altitude and inertial measurements;
- tracks relative altitude;
- maintains the highest confirmed altitude as peak altitude;
- evaluates conditions associated with apogee detection.

### APOGEE

Entered after the implemented apogee criteria have been confirmed.

Principal behaviour:

- confirms the apogee event;
- initiates the recovery-control sequence;
- commands recovery-servo actuation.

### DESCENT

Represents the post-apogee descent phase.

Principal behaviour:

- recovery output remains active as required;
- sensor acquisition continues;
- landing-detection conditions are evaluated.

### LANDED

Entered after the implemented landing-confirmation conditions have been
satisfied.

Principal behaviour:

- confirms completion of the simulated flight sequence;
- activates post-landing visual indication;
- activates audible recovery-beacon behaviour;
- displays final system/recovery status.

## Transition Sequence

The intended transition order is:

IDLE
  ↓ User ARM command
ARMED
  ↓ Launch conditions confirmed
ASCENT
  ↓ Apogee conditions confirmed
APOGEE
  ↓ Recovery sequence
DESCENT
  ↓ Landing conditions confirmed
LANDED

## Design Principle

State transitions are based on defined event-detection and confirmation
conditions rather than individual instantaneous sensor measurements.

This reduces the probability of unintended transitions caused by isolated
measurement fluctuations.

## Verification Status

The complete state sequence was reproduced during repeated integrated
bench testing.

Five final integrated test sequences completed successfully:

IDLE → ARMED → ASCENT → APOGEE → DESCENT → LANDED

FC_V1 is therefore classified as an integrated functional prototype that has
been bench verified.

This verification does not constitute flight qualification.
