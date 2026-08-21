// ============================================================
// ROSHAN AEROSPACE INDUSTRIES
// FLIGHT COMPUTER V1
//
// Firmware:
// FC_V1_v1.6.3_RobustBenchLogic
//
// PURPOSE:
// Robust hand-bench testing with reduced false triggers.
//
// IMPORTANT:
// This is still BENCH firmware.
// Thresholds are NOT flight-qualified.
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================
// OPERATING MODE
// ============================================================

enum OperatingMode
{
  BENCH_MODE,
  OPERATIONAL_MODE
};

const OperatingMode OPERATING_MODE = BENCH_MODE;

// ============================================================
// PIN MAP
// ============================================================

#define SDA_PIN       21
#define SCL_PIN       22

#define SERVO_PIN     18
#define BUZZER_PIN    25

#define LED_GREEN     26
#define LED_YELLOW    27
#define LED_BLUE      32
#define LED_RED       33

#define IMU_ADDR      0x68

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

// ============================================================
// SERVO
// ============================================================

#define SERVO_STOWED    0
#define SERVO_DEPLOYED  90

// ============================================================
// ROBUST BENCH PROFILE
// ============================================================

// ---------------- LAUNCH ----------------

// A little stronger than previous bench version
const float BENCH_LAUNCH_G_THRESHOLD = 1.20;

// Must remain above threshold for multiple samples
const int BENCH_LAUNCH_CONFIRM_SAMPLES = 2;

// Board must also move upward relative to ARM reference
const float BENCH_MIN_LAUNCH_ALT_GAIN = 0.12;

// ---------------- APOGEE ----------------

// Apogee logic is disabled until this much altitude
// has been gained above launch reference.
const float BENCH_MIN_ALTITUDE_FOR_APOGEE = 0.45;

// Must drop meaningfully from recorded peak
const float BENCH_APOGEE_DROP_THRESHOLD = 0.15;

// More confirmation than old version
const int BENCH_APOGEE_CONFIRM_SAMPLES = 4;

// Minimum time after launch before apogee can be evaluated
const unsigned long BENCH_MIN_ASCENT_TIME = 700;

// ---------------- LANDING ----------------

const float BENCH_LANDING_ALT_CHANGE_MAX = 0.08;

const float BENCH_LANDING_G_MIN = 0.80;
const float BENCH_LANDING_G_MAX = 1.20;

const int BENCH_LANDING_CONFIRM_SAMPLES = 10;

// Prevent instant landing after descent begins
const unsigned long MIN_DESCENT_TIME_FOR_LANDING = 1500;

// ============================================================
// BMP OUTLIER REJECTION
// ============================================================

// For hand testing, a sudden >3 m change between consecutive
// 100 ms samples is obviously invalid.
const float MAX_ALTITUDE_STEP = 3.0;

// ============================================================
// TIMING / FILTERING
// ============================================================

const unsigned long SENSOR_INTERVAL = 100;

const float ALTITUDE_FILTER_ALPHA = 0.80;

// ============================================================
// RECOVERY BEACON
// ============================================================

const unsigned long BEACON_ON_TIME  = 120;
const unsigned long BEACON_OFF_TIME = 900;

// ============================================================
// OBJECTS
// ============================================================

Adafruit_BMP280 bmp;

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

Servo deploymentServo;

// ============================================================
// FLIGHT STATES
// ============================================================

enum FlightState
{
  BOOT,
  SELF_TEST,
  CALIBRATION,
  IDLE,
  ARMED,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED,
  FAULT
};

FlightState flightState = BOOT;

// ============================================================
// SYSTEM HEALTH
// ============================================================

bool oledHealthy = false;
bool bmpHealthy = false;
bool imuHealthy = false;
bool systemHealthy = false;

// ============================================================
// SENSOR VARIABLES
// ============================================================

float temperature = 0.0;
float pressure = 0.0;

float absoluteAltitude = 0.0;
float relativeAltitude = 0.0;

float filteredAltitude = 0.0;
float previousFilteredAltitude = 0.0;

float lastValidRelativeAltitude = 0.0;

float baseAltitude = 0.0;
float altitudeChange = 0.0;

// ============================================================
// IMU VARIABLES
// ============================================================

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float accelerationMagnitude = 0.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

// ============================================================
// FLIGHT LOGIC VARIABLES
// ============================================================

int launchCounter = 0;
int apogeeCounter = 0;
int landingCounter = 0;

float armReferenceAltitude = 0.0;
float launchReferenceAltitude = 0.0;

float peakAltitude = 0.0;
float dropFromPeak = 0.0;

bool apogeeGateOpened = false;

unsigned long launchTime = 0;
unsigned long apogeeTime = 0;
unsigned long descentStartTime = 0;
unsigned long landingTime = 0;

unsigned long timeSinceLaunch = 0;

// ============================================================
// RECOVERY
// ============================================================

bool recoveryDeployed = false;

// ============================================================
// BUZZER
// ============================================================

bool buzzerActive = false;

unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;

bool beaconOn = false;
unsigned long beaconTimer = 0;

// ============================================================
// LED
// ============================================================

bool ledBlinkState = false;
unsigned long ledBlinkTimer = 0;

const unsigned long LED_BLINK_INTERVAL = 500;

// ============================================================
// SENSOR TIMER
// ============================================================

unsigned long lastSensorUpdate = 0;

// ============================================================
// IMU LOW LEVEL
// ============================================================

void imuWrite(byte reg, byte value)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t imuRead16(byte reg)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDR, 2);

  if (Wire.available() >= 2)
  {
    return (int16_t)(
      (Wire.read() << 8) |
      Wire.read()
    );
  }

  return 0;
}

// ============================================================
// STATE NAME
// ============================================================

const char* getStateName()
{
  switch (flightState)
  {
    case BOOT:        return "BOOT";
    case SELF_TEST:   return "SELFTEST";
    case CALIBRATION: return "CAL";
    case IDLE:        return "IDLE";
    case ARMED:       return "ARMED";
    case ASCENT:      return "ASCENT";
    case APOGEE:      return "APOGEE";
    case DESCENT:     return "DESCENT";
    case LANDED:      return "LANDED";
    case FAULT:       return "FAULT";
    default:          return "UNKNOWN";
  }
}

// ============================================================
// LED CONTROL
// ============================================================

void initializeLEDs()
{
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
}

void updateStatusLEDs()
{
  if (
    millis() - ledBlinkTimer >=
    LED_BLINK_INTERVAL
  )
  {
    ledBlinkTimer = millis();
    ledBlinkState = !ledBlinkState;
  }

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);

  switch (flightState)
  {
    case BOOT:
    case SELF_TEST:
    case CALIBRATION:

      digitalWrite(
        LED_YELLOW,
        ledBlinkState
      );

      break;

    case IDLE:

      digitalWrite(
        LED_GREEN,
        HIGH
      );

      break;

    case ARMED:

      digitalWrite(
        LED_YELLOW,
        HIGH
      );

      break;

    case ASCENT:

      digitalWrite(
        LED_BLUE,
        HIGH
      );

      break;

    case APOGEE:

      digitalWrite(
        LED_BLUE,
        HIGH
      );

      digitalWrite(
        LED_RED,
        HIGH
      );

      break;

    case DESCENT:

      digitalWrite(
        LED_BLUE,
        ledBlinkState
      );

      break;

    case LANDED:

      digitalWrite(
        LED_GREEN,
        HIGH
      );

      digitalWrite(
        LED_RED,
        ledBlinkState
      );

      break;

    case FAULT:

      digitalWrite(
        LED_RED,
        HIGH
      );

      break;
  }
}

// ============================================================
// BUZZER
// ============================================================

void startBuzzer(unsigned long duration)
{
  digitalWrite(BUZZER_PIN, HIGH);

  buzzerActive = true;
  buzzerStartTime = millis();
  buzzerDuration = duration;
}

void updateBuzzer()
{
  if (
    buzzerActive &&
    millis() - buzzerStartTime >=
    buzzerDuration
  )
  {
    digitalWrite(BUZZER_PIN, LOW);

    buzzerActive = false;
  }
}

// ============================================================
// RECOVERY BEACON
// ============================================================

void updateRecoveryBeacon()
{
  if (flightState != LANDED)
  {
    return;
  }

  unsigned long now = millis();

  if (!beaconOn)
  {
    if (
      now - beaconTimer >=
      BEACON_OFF_TIME
    )
    {
      digitalWrite(BUZZER_PIN, HIGH);

      beaconOn = true;
      beaconTimer = now;
    }
  }
  else
  {
    if (
      now - beaconTimer >=
      BEACON_ON_TIME
    )
    {
      digitalWrite(BUZZER_PIN, LOW);

      beaconOn = false;
      beaconTimer = now;
    }
  }
}

// ============================================================
// SELF TEST
// ============================================================

bool performSelfTest()
{
  flightState = SELF_TEST;

  Serial.println();
  Serial.println("======================");
  Serial.println(" STARTUP SELF TEST");
  Serial.println("======================");

  oledHealthy =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    );

  Serial.println(
    oledHealthy ?
    "[PASS] OLED" :
    "[FAIL] OLED"
  );

  bmpHealthy = false;

  if (bmp.begin(0x76))
  {
    bmpHealthy = true;

    Serial.println(
      "[PASS] BMP280 @ 0x76"
    );
  }
  else if (bmp.begin(0x77))
  {
    bmpHealthy = true;

    Serial.println(
      "[PASS] BMP280 @ 0x77"
    );
  }
  else
  {
    Serial.println(
      "[FAIL] BMP280"
    );
  }

  Wire.beginTransmission(IMU_ADDR);

  imuHealthy =
    (Wire.endTransmission() == 0);

  if (imuHealthy)
  {
    Serial.println(
      "[PASS] IMU @ 0x68"
    );

    imuWrite(
      0x6B,
      0x00
    );
  }
  else
  {
    Serial.println(
      "[FAIL] IMU"
    );
  }

  systemHealthy =
    oledHealthy &&
    bmpHealthy &&
    imuHealthy;

  if (systemHealthy)
  {
    Serial.println(
      "SELF TEST: PASS"
    );

    return true;
  }

  Serial.println(
    "SELF TEST: FAILED"
  );

  return false;
}

// ============================================================
// INITIALIZATION
// ============================================================

void initializeHardware()
{
  flightState = BOOT;

  Serial.println();
  Serial.println("================================");
  Serial.println(" ROSHAN AEROSPACE INDUSTRIES");
  Serial.println(" FLIGHT COMPUTER V1");
  Serial.println(" Firmware V1.6.3");
  Serial.println("================================");

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  initializeLEDs();

  deploymentServo.setPeriodHertz(50);

  deploymentServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  deploymentServo.write(
    SERVO_STOWED
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  if (!performSelfTest())
  {
    flightState = FAULT;
    return;
  }

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    0,
    0
  );

  display.println(
    "RAI FLIGHT COMPUTER"
  );

  display.println(
    "V1.6.3"
  );

  display.println();

  display.println(
    "SELF TEST: PASS"
  );

  display.display();

  startBuzzer(150);

  delay(500);
}

// ============================================================
// CALIBRATION
// ============================================================

void calibrateSensors()
{
  if (!systemHealthy)
  {
    flightState = FAULT;
    return;
  }

  flightState = CALIBRATION;

  display.clearDisplay();

  display.setCursor(0, 0);

  display.println("CALIBRATING...");
  display.println();
  display.println("KEEP BOARD STILL");

  display.display();

  const int samples = 100;

  float altitudeSum = 0.0;

  long gxSum = 0;
  long gySum = 0;
  long gzSum = 0;

  for (
    int i = 0;
    i < samples;
    i++
  )
  {
    altitudeSum +=
      bmp.readAltitude(
        1013.25
      );

    gxSum +=
      imuRead16(0x43);

    gySum +=
      imuRead16(0x45);

    gzSum +=
      imuRead16(0x47);

    delay(20);
  }

  baseAltitude =
    altitudeSum /
    samples;

  gyroOffsetX =
    gxSum /
    (float)samples;

  gyroOffsetY =
    gySum /
    (float)samples;

  gyroOffsetZ =
    gzSum /
    (float)samples;

  filteredAltitude = 0.0;
  previousFilteredAltitude = 0.0;

  lastValidRelativeAltitude = 0.0;

  flightState = IDLE;

  Serial.println(
    "CALIBRATION COMPLETE"
  );

  Serial.println(
    "STATE -> IDLE"
  );
}

// ============================================================
// SENSOR READING WITH OUTLIER REJECTION
// ============================================================

void readSensors()
{
  // ---------------- BMP280 ----------------

  temperature =
    bmp.readTemperature();

  pressure =
    bmp.readPressure() /
    100.0F;

  float newAbsoluteAltitude =
    bmp.readAltitude(
      1013.25
    );

  float newRelativeAltitude =
    newAbsoluteAltitude -
    baseAltitude;

  // ----------------------------------------------------------
  // Reject impossible barometric jumps
  // ----------------------------------------------------------

  if (
    fabs(
      newRelativeAltitude -
      lastValidRelativeAltitude
    )
    <=
    MAX_ALTITUDE_STEP
  )
  {
    absoluteAltitude =
      newAbsoluteAltitude;

    relativeAltitude =
      newRelativeAltitude;

    lastValidRelativeAltitude =
      newRelativeAltitude;
  }
  else
  {
    Serial.print(
      "BMP OUTLIER REJECTED: "
    );

    Serial.println(
      newRelativeAltitude,
      2
    );
  }

  previousFilteredAltitude =
    filteredAltitude;

  filteredAltitude =
    (
      ALTITUDE_FILTER_ALPHA *
      filteredAltitude
    )
    +
    (
      (1.0 - ALTITUDE_FILTER_ALPHA) *
      relativeAltitude
    );

  altitudeChange =
    fabs(
      filteredAltitude -
      previousFilteredAltitude
    );

  // ---------------- IMU ----------------

  int16_t axRaw = imuRead16(0x3B);
  int16_t ayRaw = imuRead16(0x3D);
  int16_t azRaw = imuRead16(0x3F);

  int16_t gxRaw = imuRead16(0x43);
  int16_t gyRaw = imuRead16(0x45);
  int16_t gzRaw = imuRead16(0x47);

  accelX =
    axRaw /
    16384.0;

  accelY =
    ayRaw /
    16384.0;

  accelZ =
    azRaw /
    16384.0;

  gyroX =
    (
      gxRaw -
      gyroOffsetX
    )
    /
    131.0;

  gyroY =
    (
      gyRaw -
      gyroOffsetY
    )
    /
    131.0;

  gyroZ =
    (
      gzRaw -
      gyroOffsetZ
    )
    /
    131.0;

  accelerationMagnitude =
    sqrt(
      accelX * accelX +
      accelY * accelY +
      accelZ * accelZ
    );
}

// ============================================================
// RECOVERY
// ============================================================

void deployRecovery()
{
  if (recoveryDeployed)
  {
    return;
  }

  if (
    flightState != APOGEE &&
    flightState != DESCENT
  )
  {
    return;
  }

  recoveryDeployed = true;

  deploymentServo.write(
    SERVO_DEPLOYED
  );

  startBuzzer(300);

  Serial.println();
  Serial.println("=======================");
  Serial.println(" RECOVERY DEPLOYED");
  Serial.println("=======================");
}

// ============================================================
// LAUNCH DETECTION
// ============================================================

void detectLaunch()
{
  if (flightState != ARMED)
  {
    launchCounter = 0;
    return;
  }

  float altitudeGain =
    filteredAltitude -
    armReferenceAltitude;

  bool accelerationOK =
    accelerationMagnitude >=
    BENCH_LAUNCH_G_THRESHOLD;

  bool altitudeGainOK =
    altitudeGain >=
    BENCH_MIN_LAUNCH_ALT_GAIN;

  if (
    accelerationOK &&
    altitudeGainOK
  )
  {
    launchCounter++;
  }
  else
  {
    launchCounter = 0;
  }

  if (
    launchCounter >=
    BENCH_LAUNCH_CONFIRM_SAMPLES
  )
  {
    flightState = ASCENT;

    launchTime = millis();

    launchReferenceAltitude =
      filteredAltitude;

    peakAltitude =
      filteredAltitude;

    dropFromPeak = 0.0;

    launchCounter = 0;
    apogeeCounter = 0;

    apogeeGateOpened = false;

    startBuzzer(100);

    Serial.println();
    Serial.println(
      "LAUNCH DETECTED"
    );

    Serial.println(
      "STATE -> ASCENT"
    );
  }
}

// ============================================================
// APOGEE DETECTION
// ============================================================

void detectApogee()
{
  if (flightState != ASCENT)
  {
    return;
  }

  timeSinceLaunch =
    millis() -
    launchTime;

  if (
    filteredAltitude >
    peakAltitude
  )
  {
    peakAltitude =
      filteredAltitude;
  }

  float altitudeAboveLaunch =
    filteredAltitude -
    launchReferenceAltitude;

  // ----------------------------------------------------------
  // Gate apogee detection until genuine climb occurred
  // ----------------------------------------------------------

  if (!apogeeGateOpened)
  {
    if (
      altitudeAboveLaunch >=
      BENCH_MIN_ALTITUDE_FOR_APOGEE
    )
    {
      apogeeGateOpened = true;

      Serial.println(
        "APOGEE GATE OPEN"
      );
    }
    else
    {
      return;
    }
  }

  if (
    timeSinceLaunch <
    BENCH_MIN_ASCENT_TIME
  )
  {
    return;
  }

  dropFromPeak =
    peakAltitude -
    filteredAltitude;

  if (
    dropFromPeak >=
    BENCH_APOGEE_DROP_THRESHOLD
  )
  {
    apogeeCounter++;
  }
  else
  {
    apogeeCounter = 0;
  }

  if (
    apogeeCounter >=
    BENCH_APOGEE_CONFIRM_SAMPLES
  )
  {
    flightState = APOGEE;

    apogeeTime = millis();

    apogeeCounter = 0;

    Serial.println();
    Serial.println(
      "APOGEE DETECTED"
    );

    Serial.print(
      "PEAK ALTITUDE: "
    );

    Serial.print(
      peakAltitude,
      2
    );

    Serial.println(" m");

    deployRecovery();
  }
}

// ============================================================
// APOGEE -> DESCENT
// ============================================================

void updateApogeeState()
{
  if (
    flightState != APOGEE
  )
  {
    return;
  }

  if (
    millis() -
    apogeeTime >=
    500
  )
  {
    flightState = DESCENT;

    descentStartTime =
      millis();

    landingCounter = 0;

    Serial.println(
      "STATE -> DESCENT"
    );
  }
}

// ============================================================
// LANDING DETECTION
// ============================================================

void detectLanding()
{
  if (
    flightState != DESCENT
  )
  {
    landingCounter = 0;
    return;
  }

  if (
    millis() -
    descentStartTime <
    MIN_DESCENT_TIME_FOR_LANDING
  )
  {
    landingCounter = 0;
    return;
  }

  bool altitudeStable =
    altitudeChange <=
    BENCH_LANDING_ALT_CHANGE_MAX;

  bool accelerationStable =
    (
      accelerationMagnitude >=
      BENCH_LANDING_G_MIN
    )
    &&
    (
      accelerationMagnitude <=
      BENCH_LANDING_G_MAX
    );

  if (
    altitudeStable &&
    accelerationStable
  )
  {
    landingCounter++;
  }
  else
  {
    landingCounter = 0;
  }

  if (
    landingCounter >=
    BENCH_LANDING_CONFIRM_SAMPLES
  )
  {
    flightState = LANDED;

    landingTime = millis();

    beaconTimer = millis();
    beaconOn = false;

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerActive = false;

    landingCounter = 0;

    Serial.println();
    Serial.println("=======================");
    Serial.println(" LANDING DETECTED");
    Serial.println(" STATE -> LANDED");
    Serial.println("=======================");
  }
}

// ============================================================
// RESET
// ============================================================

void resetFlightComputer()
{
  if (flightState == FAULT)
  {
    return;
  }

  flightState = IDLE;

  launchCounter = 0;
  apogeeCounter = 0;
  landingCounter = 0;

  launchTime = 0;
  apogeeTime = 0;
  descentStartTime = 0;
  landingTime = 0;

  timeSinceLaunch = 0;

  peakAltitude =
    filteredAltitude;

  dropFromPeak = 0.0;

  apogeeGateOpened = false;

  recoveryDeployed = false;

  deploymentServo.write(
    SERVO_STOWED
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  buzzerActive = false;

  beaconOn = false;
  beaconTimer = 0;

  Serial.println();
  Serial.println(
    "FLIGHT COMPUTER RESET"
  );
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void processSerialCommands()
{
  if (!Serial.available())
  {
    return;
  }

  String command =
    Serial.readStringUntil(
      '\n'
    );

  command.trim();
  command.toUpperCase();

  // ---------------- ARM ----------------

  if (command == "ARM")
  {
    if (
      flightState != IDLE
    )
    {
      Serial.println(
        "ARM BLOCKED: NOT IDLE"
      );

      return;
    }

    if (!systemHealthy)
    {
      Serial.println(
        "ARM BLOCKED: SYSTEM NOT HEALTHY"
      );

      return;
    }

    flightState = ARMED;

    armReferenceAltitude =
      filteredAltitude;

    launchCounter = 0;
    apogeeCounter = 0;
    landingCounter = 0;

    apogeeGateOpened = false;

    startBuzzer(150);

    Serial.println();
    Serial.println("SYSTEM ARMED");

    Serial.print(
      "ARM ALT REF: "
    );

    Serial.println(
      armReferenceAltitude,
      2
    );

    return;
  }

  // ---------------- DISARM ----------------

  if (command == "DISARM")
  {
    if (
      flightState == ARMED
    )
    {
      flightState = IDLE;

      Serial.println(
        "SYSTEM DISARMED"
      );
    }

    return;
  }

  // ---------------- RESET ----------------

  if (command == "RESET")
  {
    resetFlightComputer();

    return;
  }

  // ---------------- STATUS ----------------

  if (command == "STATUS")
  {
    Serial.println();
    Serial.println(
      "===== SYSTEM STATUS ====="
    );

    Serial.print("STATE: ");
    Serial.println(
      getStateName()
    );

    Serial.print(
      "ALT: "
    );

    Serial.println(
      filteredAltitude,
      2
    );

    Serial.print(
      "ARM REF: "
    );

    Serial.println(
      armReferenceAltitude,
      2
    );

    Serial.print(
      "PEAK: "
    );

    Serial.println(
      peakAltitude,
      2
    );

    Serial.print(
      "G: "
    );

    Serial.println(
      accelerationMagnitude,
      2
    );

    Serial.print(
      "RECOVERY: "
    );

    Serial.println(
      recoveryDeployed ?
      "DEPLOYED" :
      "STOWED"
    );

    Serial.print(
      "APOGEE GATE: "
    );

    Serial.println(
      apogeeGateOpened ?
      "OPEN" :
      "LOCKED"
    );

    Serial.println(
      "========================="
    );

    return;
  }
}

// ============================================================
// OLED
// ============================================================

void updateDisplay()
{
  if (!oledHealthy)
  {
    return;
  }

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    0,
    0
  );

  display.println(
    "RAI FLIGHT COMPUTER"
  );

  display.print("STATE:");
  display.println(
    getStateName()
  );

  display.print("ALT:");
  display.print(
    filteredAltitude,
    2
  );
  display.println("m");

  display.print("PEAK:");
  display.print(
    peakAltitude,
    2
  );
  display.println("m");

  display.print("G:");
  display.println(
    accelerationMagnitude,
    2
  );

  display.print("REC:");

  display.println(
    recoveryDeployed ?
    "DEPLOYED" :
    "STOWED"
  );

  if (
    flightState ==
    LANDED
  )
  {
    display.println(
      "BEACON:ACTIVE"
    );
  }

  display.display();
}

// ============================================================
// TELEMETRY
// ============================================================

void printTelemetry()
{
  Serial.print("STATE=");
  Serial.print(getStateName());

  Serial.print(" | ALT=");
  Serial.print(
    filteredAltitude,
    2
  );

  Serial.print("m | ARMREF=");
  Serial.print(
    armReferenceAltitude,
    2
  );

  Serial.print("m | PEAK=");
  Serial.print(
    peakAltitude,
    2
  );

  Serial.print("m | DROP=");
  Serial.print(
    dropFromPeak,
    2
  );

  Serial.print("m | dALT=");
  Serial.print(
    altitudeChange,
    3
  );

  Serial.print("m | G=");
  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.print(" | APO_GATE=");
  Serial.print(
    apogeeGateOpened ?
    "OPEN" :
    "LOCKED"
  );

  Serial.print(" | LAND=");
  Serial.print(
    landingCounter
  );

  Serial.print("/");
  Serial.print(
    BENCH_LANDING_CONFIRM_SAMPLES
  );

  Serial.print(" | REC=");

  Serial.println(
    recoveryDeployed ?
    "DEPLOYED" :
    "STOWED"
  );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  initializeHardware();

  if (
    flightState ==
    FAULT
  )
  {
    updateStatusLEDs();

    Serial.println(
      "*** SYSTEM FAULT ***"
    );

    return;
  }

  calibrateSensors();

  updateStatusLEDs();

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    " FC_V1 V1.6.3 ROBUST BENCH"
  );

  Serial.println(
    "================================"
  );

  Serial.println();
  Serial.println(
    "COMMANDS:"
  );

  Serial.println(
    "ARM"
  );

  Serial.println(
    "DISARM"
  );

  Serial.println(
    "RESET"
  );

  Serial.println(
    "STATUS"
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  updateStatusLEDs();

  if (
    flightState ==
    FAULT
  )
  {
    delay(50);
    return;
  }

  processSerialCommands();

  updateBuzzer();

  updateRecoveryBeacon();

  if (
    millis() -
    lastSensorUpdate >=
    SENSOR_INTERVAL
  )
  {
    lastSensorUpdate =
      millis();

    readSensors();

    detectLaunch();

    detectApogee();

    updateApogeeState();

    detectLanding();

    if (
      flightState == ASCENT ||
      flightState == APOGEE ||
      flightState == DESCENT ||
      flightState == LANDED
    )
    {
      timeSinceLaunch =
        millis() -
        launchTime;
    }

    updateDisplay();

    printTelemetry();
  }
}
