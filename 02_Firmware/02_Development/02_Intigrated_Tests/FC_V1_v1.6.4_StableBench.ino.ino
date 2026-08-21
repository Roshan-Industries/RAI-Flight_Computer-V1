// ============================================================
// ROSHAN AEROSPACE INDUSTRIES
// FLIGHT COMPUTER V1
//
// Firmware:
// FC_V1_v1.6.4_StableBench
//
// PURPOSE:
// Stable, repeatable HAND BENCH TEST firmware.
//
// Hardware:
// ESP32
// BMP280
// MPU6050-compatible IMU
// SSD1306 OLED
// Servo
// Active Buzzer
// 4 Status LEDs
//
// RESET BUTTON NOT INTEGRATED YET.
//
// IMPORTANT:
// THIS IS BENCH TEST FIRMWARE.
// THESE THRESHOLDS ARE NOT FLIGHT-QUALIFIED.
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>

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
// SENSOR LOOP
// ============================================================

const unsigned long SENSOR_INTERVAL = 100;

// ============================================================
// ALTITUDE FILTER
// ============================================================

// Moderate filtering.
// 0.75 = 75% previous + 25% new measurement.
const float ALT_FILTER_ALPHA = 0.75;

// Reject absurd sudden altitude changes.
const float MAX_VALID_ALTITUDE_STEP = 2.0;

// Plausible BMP pressure range for bench operation.
const float MIN_VALID_PRESSURE = 850.0;
const float MAX_VALID_PRESSURE = 1100.0;

// ============================================================
// LAUNCH DETECTION
// ============================================================

// Hand acceleration must exceed this.
const float LAUNCH_ACCEL_THRESHOLD = 1.25;

// Must happen for 2 samples.
const int LAUNCH_ACCEL_CONFIRM = 2;

// After an acceleration candidate,
// altitude must rise this much.
const float LAUNCH_ALT_GAIN_REQUIRED = 0.12;

// Time allowed after acceleration candidate
// for altitude confirmation.
const unsigned long LAUNCH_CONFIRM_WINDOW = 2500;

// ============================================================
// APOGEE DETECTION
// ============================================================

// Must actually climb this far above launch altitude
// before apogee detection is even allowed.
const float MIN_CLIMB_BEFORE_APOGEE = 0.40;

// Must fall this far below peak.
const float APOGEE_DROP_REQUIRED = 0.18;

// Sustained falling samples.
const int APOGEE_CONFIRM_SAMPLES = 4;

// Ignore apogee logic immediately after launch.
const unsigned long MIN_ASCENT_TIME = 1000;

// ============================================================
// LANDING DETECTION
// ============================================================

// No landing allowed immediately after descent starts.
const unsigned long MIN_DESCENT_TIME = 1500;

// Store about 1 second of altitude samples.
const int LANDING_WINDOW_SIZE = 10;

// Maximum altitude spread across the window
// while considered stationary.
const float LANDING_ALT_RANGE = 0.12;

// Acceleration should approximately equal gravity.
const float LANDING_G_MIN = 0.80;
const float LANDING_G_MAX = 1.20;

// Need repeated stable windows.
const int LANDING_STABLE_CONFIRM = 5;

// ============================================================
// RECOVERY BEACON
// ============================================================

const unsigned long BEACON_ON_TIME = 120;
const unsigned long BEACON_OFF_TIME = 900;

// ============================================================
// LED BLINK
// ============================================================

const unsigned long LED_BLINK_INTERVAL = 500;

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
// HEALTH FLAGS
// ============================================================

bool oledHealthy = false;
bool bmpHealthy = false;
bool imuHealthy = false;
bool systemHealthy = false;

// ============================================================
// BMP VARIABLES
// ============================================================

float temperature = 0.0;
float pressure = 0.0;

float baseAltitude = 0.0;

float rawRelativeAltitude = 0.0;
float filteredAltitude = 0.0;

float lastAcceptedAltitude = 0.0;

// ============================================================
// IMU VARIABLES
// ============================================================

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

float accelerationMagnitude = 0.0;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

// ============================================================
// FLIGHT VARIABLES
// ============================================================

float armAltitude = 0.0;
float launchAltitude = 0.0;
float peakAltitude = 0.0;
float dropFromPeak = 0.0;

unsigned long launchTime = 0;
unsigned long apogeeTime = 0;
unsigned long descentStartTime = 0;

bool recoveryDeployed = false;

// ============================================================
// LAUNCH CANDIDATE LOGIC
// ============================================================

int launchAccelCounter = 0;

bool launchCandidate = false;

unsigned long launchCandidateTime = 0;

float launchCandidateAltitude = 0.0;

// ============================================================
// APOGEE LOGIC
// ============================================================

bool apogeeGateOpen = false;

int apogeeCounter = 0;

// ============================================================
// LANDING WINDOW
// ============================================================

float landingAltitudeWindow[LANDING_WINDOW_SIZE];

int landingWindowIndex = 0;
int landingWindowCount = 0;

int stableLandingCounter = 0;

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

// ============================================================
// MAIN TIMER
// ============================================================

unsigned long lastSensorUpdate = 0;

// ============================================================
// IMU LOW-LEVEL FUNCTIONS
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
// LED INITIALIZATION
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

// ============================================================
// LED STATE CONTROL
// ============================================================

void updateStatusLEDs()
{
  if (
    millis() - ledBlinkTimer >=
    LED_BLINK_INTERVAL
  )
  {
    ledBlinkTimer = millis();

    ledBlinkState =
      !ledBlinkState;
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

  // OLED
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

  // BMP
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

  // IMU
  Wire.beginTransmission(IMU_ADDR);

  imuHealthy =
    (
      Wire.endTransmission()
      == 0
    );

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
// HARDWARE INITIALIZATION
// ============================================================

void initializeHardware()
{
  flightState = BOOT;

  Serial.println();
  Serial.println("================================");
  Serial.println(" ROSHAN AEROSPACE INDUSTRIES");
  Serial.println(" FLIGHT COMPUTER V1");
  Serial.println(" Firmware V1.6.4");
  Serial.println("================================");

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  initializeLEDs();

  // Servo
  deploymentServo.setPeriodHertz(50);

  deploymentServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  deploymentServo.write(
    SERVO_STOWED
  );

  // Buzzer
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
    "V1.6.4"
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

  display.setCursor(
    0,
    0
  );

  display.println(
    "CALIBRATING..."
  );

  display.println();

  display.println(
    "KEEP STILL"
  );

  display.display();

  Serial.println();
  Serial.println(
    "CALIBRATION START"
  );

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
    float a =
      bmp.readAltitude(
        1013.25
      );

    altitudeSum += a;

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

  rawRelativeAltitude = 0.0;

  filteredAltitude = 0.0;

  lastAcceptedAltitude = 0.0;

  flightState = IDLE;

  Serial.println(
    "CALIBRATION COMPLETE"
  );

  Serial.println(
    "STATE -> IDLE"
  );
}

// ============================================================
// SENSOR READING
// ============================================================

void readSensors()
{
  // ==========================================================
  // BMP280
  // ==========================================================

  temperature =
    bmp.readTemperature();

  float newPressure =
    bmp.readPressure() /
    100.0F;

  float newAbsoluteAltitude =
    bmp.readAltitude(
      1013.25
    );

  float newRelativeAltitude =
    newAbsoluteAltitude -
    baseAltitude;

  bool pressureValid =
    (
      newPressure >=
      MIN_VALID_PRESSURE
    )
    &&
    (
      newPressure <=
      MAX_VALID_PRESSURE
    );

  bool altitudeStepValid =
    fabs(
      newRelativeAltitude -
      lastAcceptedAltitude
    )
    <=
    MAX_VALID_ALTITUDE_STEP;

  if (
    pressureValid &&
    altitudeStepValid &&
    !isnan(newRelativeAltitude)
  )
  {
    pressure =
      newPressure;

    rawRelativeAltitude =
      newRelativeAltitude;

    lastAcceptedAltitude =
      newRelativeAltitude;
  }

  else
  {
    Serial.print(
      "BMP SAMPLE REJECTED | P="
    );

    Serial.print(
      newPressure
    );

    Serial.print(
      " | ALT="
    );

    Serial.println(
      newRelativeAltitude
    );
  }

  filteredAltitude =
    (
      ALT_FILTER_ALPHA *
      filteredAltitude
    )
    +
    (
      (1.0 - ALT_FILTER_ALPHA) *
      rawRelativeAltitude
    );

  // ==========================================================
  // IMU
  // ==========================================================

  int16_t axRaw =
    imuRead16(0x3B);

  int16_t ayRaw =
    imuRead16(0x3D);

  int16_t azRaw =
    imuRead16(0x3F);

  int16_t gxRaw =
    imuRead16(0x43);

  int16_t gyRaw =
    imuRead16(0x45);

  int16_t gzRaw =
    imuRead16(0x47);

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
// RECOVERY DEPLOYMENT
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
  Serial.println(
    "======================="
  );

  Serial.println(
    " RECOVERY DEPLOYED"
  );

  Serial.println(
    "======================="
  );
}

// ============================================================
// LAUNCH DETECTION
// ============================================================

void detectLaunch()
{
  if (flightState != ARMED)
  {
    launchAccelCounter = 0;

    launchCandidate = false;

    return;
  }

  // ----------------------------------------------------------
  // STAGE 1:
  // Detect acceleration event
  // ----------------------------------------------------------

  if (!launchCandidate)
  {
    if (
      accelerationMagnitude >=
      LAUNCH_ACCEL_THRESHOLD
    )
    {
      launchAccelCounter++;
    }
    else
    {
      launchAccelCounter = 0;
    }

    if (
      launchAccelCounter >=
      LAUNCH_ACCEL_CONFIRM
    )
    {
      launchCandidate = true;

      launchCandidateTime =
        millis();

      launchCandidateAltitude =
        filteredAltitude;

      launchAccelCounter = 0;

      Serial.println();
      Serial.println(
        "LAUNCH CANDIDATE"
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // STAGE 2:
  // Confirm actual upward movement
  // ----------------------------------------------------------

  float candidateAltitudeGain =
    filteredAltitude -
    launchCandidateAltitude;

  if (
    candidateAltitudeGain >=
    LAUNCH_ALT_GAIN_REQUIRED
  )
  {
    flightState =
      ASCENT;

    launchTime =
      millis();

    launchAltitude =
      filteredAltitude;

    peakAltitude =
      filteredAltitude;

    launchCandidate =
      false;

    apogeeGateOpen =
      false;

    apogeeCounter =
      0;

    startBuzzer(100);

    Serial.println();
    Serial.println(
      "LAUNCH CONFIRMED"
    );

    Serial.println(
      "STATE -> ASCENT"
    );

    return;
  }

  // ----------------------------------------------------------
  // Candidate timeout
  // ----------------------------------------------------------

  if (
    millis() -
    launchCandidateTime >
    LAUNCH_CONFIRM_WINDOW
  )
  {
    launchCandidate =
      false;

    launchAccelCounter =
      0;

    Serial.println(
      "LAUNCH CANDIDATE REJECTED"
    );
  }
}

// ============================================================
// APOGEE DETECTION
// ============================================================

void detectApogee()
{
  if (
    flightState !=
    ASCENT
  )
  {
    return;
  }

  if (
    filteredAltitude >
    peakAltitude
  )
  {
    peakAltitude =
      filteredAltitude;

    apogeeCounter =
      0;
  }

  float climbAboveLaunch =
    peakAltitude -
    launchAltitude;

  // ----------------------------------------------------------
  // Do not even consider apogee until meaningful climb happened.
  // ----------------------------------------------------------

  if (!apogeeGateOpen)
  {
    if (
      climbAboveLaunch >=
      MIN_CLIMB_BEFORE_APOGEE
    )
    {
      apogeeGateOpen =
        true;

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
    millis() -
    launchTime <
    MIN_ASCENT_TIME
  )
  {
    return;
  }

  dropFromPeak =
    peakAltitude -
    filteredAltitude;

  if (
    dropFromPeak >=
    APOGEE_DROP_REQUIRED
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
    APOGEE_CONFIRM_SAMPLES
  )
  {
    flightState =
      APOGEE;

    apogeeTime =
      millis();

    apogeeCounter =
      0;

    Serial.println();
    Serial.println(
      "APOGEE DETECTED"
    );

    Serial.print(
      "PEAK="
    );

    Serial.println(
      peakAltitude,
      2
    );

    deployRecovery();
  }
}

// ============================================================
// APOGEE → DESCENT
// ============================================================

void updateApogeeState()
{
  if (
    flightState !=
    APOGEE
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
    flightState =
      DESCENT;

    descentStartTime =
      millis();

    landingWindowIndex =
      0;

    landingWindowCount =
      0;

    stableLandingCounter =
      0;

    Serial.println(
      "STATE -> DESCENT"
    );
  }
}

// ============================================================
// LANDING WINDOW
// ============================================================

void addLandingAltitudeSample(
  float altitude
)
{
  landingAltitudeWindow[
    landingWindowIndex
  ] = altitude;

  landingWindowIndex++;

  if (
    landingWindowIndex >=
    LANDING_WINDOW_SIZE
  )
  {
    landingWindowIndex =
      0;
  }

  if (
    landingWindowCount <
    LANDING_WINDOW_SIZE
  )
  {
    landingWindowCount++;
  }
}

// ============================================================
// CALCULATE ALTITUDE RANGE
// ============================================================

float getLandingAltitudeRange()
{
  if (
    landingWindowCount <
    LANDING_WINDOW_SIZE
  )
  {
    return 999.0;
  }

  float minimum =
    landingAltitudeWindow[0];

  float maximum =
    landingAltitudeWindow[0];

  for (
    int i = 1;
    i < LANDING_WINDOW_SIZE;
    i++
  )
  {
    if (
      landingAltitudeWindow[i] <
      minimum
    )
    {
      minimum =
        landingAltitudeWindow[i];
    }

    if (
      landingAltitudeWindow[i] >
      maximum
    )
    {
      maximum =
        landingAltitudeWindow[i];
    }
  }

  return maximum - minimum;
}

// ============================================================
// LANDING DETECTION
// ============================================================

void detectLanding()
{
  if (
    flightState !=
    DESCENT
  )
  {
    return;
  }

  if (
    millis() -
    descentStartTime <
    MIN_DESCENT_TIME
  )
  {
    return;
  }

  addLandingAltitudeSample(
    filteredAltitude
  );

  if (
    landingWindowCount <
    LANDING_WINDOW_SIZE
  )
  {
    return;
  }

  float altitudeRange =
    getLandingAltitudeRange();

  bool altitudeStable =
    altitudeRange <=
    LANDING_ALT_RANGE;

  bool gStable =
    (
      accelerationMagnitude >=
      LANDING_G_MIN
    )
    &&
    (
      accelerationMagnitude <=
      LANDING_G_MAX
    );

  if (
    altitudeStable &&
    gStable
  )
  {
    stableLandingCounter++;
  }

  else
  {
    stableLandingCounter = 0;
  }

  if (
    stableLandingCounter >=
    LANDING_STABLE_CONFIRM
  )
  {
    flightState =
      LANDED;

    beaconTimer =
      millis();

    beaconOn =
      false;

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerActive =
      false;

    Serial.println();
    Serial.println(
      "======================="
    );

    Serial.println(
      " LANDING DETECTED"
    );

    Serial.println(
      " STATE -> LANDED"
    );

    Serial.println(
      "======================="
    );
  }
}

// ============================================================
// RESET
// ============================================================

void resetFlightComputer()
{
  if (
    flightState ==
    FAULT
  )
  {
    return;
  }

  flightState =
    IDLE;

  launchAccelCounter =
    0;

  launchCandidate =
    false;

  apogeeCounter =
    0;

  apogeeGateOpen =
    false;

  landingWindowIndex =
    0;

  landingWindowCount =
    0;

  stableLandingCounter =
    0;

  peakAltitude =
    filteredAltitude;

  recoveryDeployed =
    false;

  deploymentServo.write(
    SERVO_STOWED
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  buzzerActive =
    false;

  beaconOn =
    false;

  beaconTimer =
    0;

  Serial.println();
  Serial.println(
    "FLIGHT COMPUTER RESET"
  );

  Serial.println(
    "STATE -> IDLE"
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

  // ==========================================================
  // ARM
  // ==========================================================

  if (
    command ==
    "ARM"
  )
  {
    if (
      flightState !=
      IDLE
    )
    {
      Serial.println(
        "ARM BLOCKED"
      );

      return;
    }

    flightState =
      ARMED;

    armAltitude =
      filteredAltitude;

    launchCandidate =
      false;

    launchAccelCounter =
      0;

    apogeeGateOpen =
      false;

    recoveryDeployed =
      false;

    startBuzzer(
      150
    );

    Serial.println();
    Serial.println(
      "SYSTEM ARMED"
    );

    Serial.print(
      "ARM ALT="
    );

    Serial.println(
      armAltitude,
      2
    );

    return;
  }

  // ==========================================================
  // DISARM
  // ==========================================================

  if (
    command ==
    "DISARM"
  )
  {
    if (
      flightState ==
      ARMED
    )
    {
      flightState =
        IDLE;

      launchCandidate =
        false;

      Serial.println(
        "SYSTEM DISARMED"
      );
    }

    return;
  }

  // ==========================================================
  // RESET
  // ==========================================================

  if (
    command ==
    "RESET"
  )
  {
    resetFlightComputer();

    return;
  }

  // ==========================================================
  // STATUS
  // ==========================================================

  if (
    command ==
    "STATUS"
  )
  {
    Serial.println();
    Serial.println(
      "===== FC STATUS ====="
    );

    Serial.print(
      "STATE: "
    );

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
      "G: "
    );

    Serial.println(
      accelerationMagnitude,
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
      "LAUNCH CANDIDATE: "
    );

    Serial.println(
      launchCandidate ?
      "YES" :
      "NO"
    );

    Serial.print(
      "APOGEE GATE: "
    );

    Serial.println(
      apogeeGateOpen ?
      "OPEN" :
      "LOCKED"
    );

    Serial.print(
      "RECOVERY: "
    );

    Serial.println(
      recoveryDeployed ?
      "DEPLOYED" :
      "STOWED"
    );

    Serial.println(
      "====================="
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

  display.print(
    "STATE:"
  );

  display.println(
    getStateName()
  );

  display.print(
    "ALT:"
  );

  display.print(
    filteredAltitude,
    2
  );

  display.println(
    "m"
  );

  display.print(
    "PEAK:"
  );

  display.print(
    peakAltitude,
    2
  );

  display.println(
    "m"
  );

  display.print(
    "G:"
  );

  display.println(
    accelerationMagnitude,
    2
  );

  display.print(
    "REC:"
  );

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
  Serial.print(
    "STATE="
  );

  Serial.print(
    getStateName()
  );

  Serial.print(
    " | ALT="
  );

  Serial.print(
    filteredAltitude,
    2
  );

  Serial.print(
    " | G="
  );

  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.print(
    " | PEAK="
  );

  Serial.print(
    peakAltitude,
    2
  );

  Serial.print(
    " | CAND="
  );

  Serial.print(
    launchCandidate ?
    "Y" :
    "N"
  );

  Serial.print(
    " | APO="
  );

  Serial.print(
    apogeeGateOpen ?
    "OPEN" :
    "LOCK"
  );

  Serial.print(
    " | LAND="
  );

  Serial.print(
    stableLandingCounter
  );

  Serial.print(
    "/"
  );

  Serial.print(
    LANDING_STABLE_CONFIRM
  );

  Serial.print(
    " | REC="
  );

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
  Serial.begin(
    115200
  );

  delay(
    1000
  );

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
    " FC_V1 V1.6.4 STABLE BENCH"
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

    updateDisplay();

    printTelemetry();
  }
}
