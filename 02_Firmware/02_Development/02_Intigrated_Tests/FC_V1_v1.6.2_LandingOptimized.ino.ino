// ============================================================
// ROSHAN AEROSPACE INDUSTRIES
// FLIGHT COMPUTER V1
//
// Firmware:
// FC_V1_v1.6.2_LandingOptimized
//
// BENCH TEST / DEVELOPMENT FIRMWARE
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
// Push button NOT integrated yet.
//
// IMPORTANT:
// Bench thresholds are intentionally designed for hand testing.
// They are NOT flight-qualified thresholds.
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
// LOCKED PIN MAP
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
// BENCH PROFILE
// ============================================================

const float BENCH_LAUNCH_G_THRESHOLD = 1.15;
const int BENCH_LAUNCH_CONFIRM_SAMPLES = 1;

const float BENCH_APOGEE_DROP_THRESHOLD = 0.08;
const int BENCH_APOGEE_CONFIRM_SAMPLES = 2;

const unsigned long BENCH_MIN_ASCENT_TIME = 300;

// ---------------- LANDING V1.6.2 ----------------
//
// More responsive for hand testing.
//

const float BENCH_LANDING_ALT_CHANGE_MAX = 0.08;

const float BENCH_LANDING_G_MIN = 0.80;
const float BENCH_LANDING_G_MAX = 1.20;

const int BENCH_LANDING_CONFIRM_SAMPLES = 8;

// Landing cannot be detected immediately after apogee.
// This prevents premature landing detection during descent.
const unsigned long MIN_DESCENT_TIME_FOR_LANDING = 1000;

// ============================================================
// OPERATIONAL PROFILE PLACEHOLDERS
// ============================================================
//
// NOT flight-qualified values.
//

const float OP_LAUNCH_G_THRESHOLD = 2.00;
const int OP_LAUNCH_CONFIRM_SAMPLES = 5;

const float OP_APOGEE_DROP_THRESHOLD = 1.00;
const int OP_APOGEE_CONFIRM_SAMPLES = 5;

const unsigned long OP_MIN_ASCENT_TIME = 2000;

const float OP_LANDING_ALT_CHANGE_MAX = 0.10;

const float OP_LANDING_G_MIN = 0.90;
const float OP_LANDING_G_MAX = 1.10;

const int OP_LANDING_CONFIRM_SAMPLES = 30;

// ============================================================
// ACTIVE PROFILE
// ============================================================

float launchGThreshold;
int launchConfirmSamples;

float apogeeDropThreshold;
int apogeeConfirmSamples;

unsigned long minimumAscentTime;

float landingAltitudeChangeMax;

float landingGMin;
float landingGMax;

int landingConfirmSamples;

// ============================================================
// TIMING / FILTERING
// ============================================================

const unsigned long SENSOR_INTERVAL = 100;

const float ALTITUDE_FILTER_ALPHA = 0.80;

// ============================================================
// RECOVERY BEACON
// ============================================================

const unsigned long BEACON_ON_TIME = 120;
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
// HEALTH
// ============================================================

bool oledHealthy = false;
bool bmpHealthy = false;
bool imuHealthy = false;
bool systemHealthy = false;

// ============================================================
// SENSOR VALUES
// ============================================================

float temperature = 0.0;
float pressure = 0.0;

float absoluteAltitude = 0.0;
float relativeAltitude = 0.0;

float filteredAltitude = 0.0;
float previousFilteredAltitude = 0.0;

float baseAltitude = 0.0;
float altitudeChange = 0.0;

// ============================================================
// IMU VALUES
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
// FLIGHT LOGIC
// ============================================================

int launchCounter = 0;
int apogeeCounter = 0;
int landingCounter = 0;

float peakAltitude = 0.0;
float dropFromPeak = 0.0;

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
// PROFILE
// ============================================================

void loadOperatingProfile()
{
  if (OPERATING_MODE == BENCH_MODE)
  {
    launchGThreshold =
      BENCH_LAUNCH_G_THRESHOLD;

    launchConfirmSamples =
      BENCH_LAUNCH_CONFIRM_SAMPLES;

    apogeeDropThreshold =
      BENCH_APOGEE_DROP_THRESHOLD;

    apogeeConfirmSamples =
      BENCH_APOGEE_CONFIRM_SAMPLES;

    minimumAscentTime =
      BENCH_MIN_ASCENT_TIME;

    landingAltitudeChangeMax =
      BENCH_LANDING_ALT_CHANGE_MAX;

    landingGMin =
      BENCH_LANDING_G_MIN;

    landingGMax =
      BENCH_LANDING_G_MAX;

    landingConfirmSamples =
      BENCH_LANDING_CONFIRM_SAMPLES;

    Serial.println("PROFILE: HAND BENCH MODE");
  }
  else
  {
    launchGThreshold =
      OP_LAUNCH_G_THRESHOLD;

    launchConfirmSamples =
      OP_LAUNCH_CONFIRM_SAMPLES;

    apogeeDropThreshold =
      OP_APOGEE_DROP_THRESHOLD;

    apogeeConfirmSamples =
      OP_APOGEE_CONFIRM_SAMPLES;

    minimumAscentTime =
      OP_MIN_ASCENT_TIME;

    landingAltitudeChangeMax =
      OP_LANDING_ALT_CHANGE_MAX;

    landingGMin =
      OP_LANDING_G_MIN;

    landingGMax =
      OP_LANDING_G_MAX;

    landingConfirmSamples =
      OP_LANDING_CONFIRM_SAMPLES;

    Serial.println("PROFILE: OPERATIONAL");
    Serial.println("WARNING: NOT FLIGHT QUALIFIED");
  }
}

// ============================================================
// LEDs
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
  digitalWrite(
    BUZZER_PIN,
    HIGH
  );

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
    digitalWrite(
      BUZZER_PIN,
      LOW
    );

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
      digitalWrite(
        BUZZER_PIN,
        HIGH
      );

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
      digitalWrite(
        BUZZER_PIN,
        LOW
      );

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
    Serial.println("[PASS] BMP280 @ 0x76");
  }
  else if (bmp.begin(0x77))
  {
    bmpHealthy = true;
    Serial.println("[PASS] BMP280 @ 0x77");
  }
  else
  {
    Serial.println("[FAIL] BMP280");
  }

  Wire.beginTransmission(IMU_ADDR);

  imuHealthy =
    (Wire.endTransmission() == 0);

  if (imuHealthy)
  {
    Serial.println("[PASS] IMU @ 0x68");

    imuWrite(
      0x6B,
      0x00
    );
  }
  else
  {
    Serial.println("[FAIL] IMU");
  }

  systemHealthy =
    oledHealthy &&
    bmpHealthy &&
    imuHealthy;

  Serial.println();

  if (systemHealthy)
  {
    Serial.println("SELF TEST: PASS");
    return true;
  }

  Serial.println("SELF TEST: FAILED");
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
  Serial.println(" Firmware V1.6.2");
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

  loadOperatingProfile();

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
    "V1.6.2"
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
    "KEEP BOARD STILL"
  );

  display.display();

  Serial.println();
  Serial.println("CALIBRATION START");

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
      bmp.readAltitude(1013.25);

    gxSum +=
      imuRead16(0x43);

    gySum +=
      imuRead16(0x45);

    gzSum +=
      imuRead16(0x47);

    if (i % 20 == 0)
    {
      digitalWrite(
        LED_YELLOW,
        !digitalRead(LED_YELLOW)
      );
    }

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

  flightState = IDLE;

  Serial.println("CALIBRATION COMPLETE");
  Serial.println("STATE -> IDLE");
}

// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  temperature =
    bmp.readTemperature();

  pressure =
    bmp.readPressure() /
    100.0F;

  absoluteAltitude =
    bmp.readAltitude(
      1013.25
    );

  relativeAltitude =
    absoluteAltitude -
    baseAltitude;

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
    (gxRaw - gyroOffsetX) /
    131.0;

  gyroY =
    (gyRaw - gyroOffsetY) /
    131.0;

  gyroZ =
    (gzRaw - gyroOffsetZ) /
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
    Serial.println(
      "DEPLOY BLOCKED: ALREADY DEPLOYED"
    );

    return;
  }

  if (
    flightState != APOGEE &&
    flightState != DESCENT
  )
  {
    Serial.println(
      "DEPLOY BLOCKED: INVALID STATE"
    );

    return;
  }

  recoveryDeployed = true;

  deploymentServo.write(
    SERVO_DEPLOYED
  );

  startBuzzer(300);

  Serial.println();
  Serial.println("=========================");
  Serial.println(" RECOVERY DEPLOYED");
  Serial.println(" SERVO -> 90 DEG");
  Serial.println("=========================");
}

// ============================================================
// LAUNCH
// ============================================================

void detectLaunch()
{
  if (flightState != ARMED)
  {
    launchCounter = 0;
    return;
  }

  if (
    accelerationMagnitude >=
    launchGThreshold
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
    launchConfirmSamples
  )
  {
    flightState = ASCENT;

    launchTime = millis();

    peakAltitude =
      filteredAltitude;

    dropFromPeak = 0.0;

    launchCounter = 0;
    apogeeCounter = 0;

    startBuzzer(100);

    Serial.println();
    Serial.println("LAUNCH DETECTED");
    Serial.println("STATE -> ASCENT");
  }
}

// ============================================================
// APOGEE
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
    timeSinceLaunch <
    minimumAscentTime
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

    dropFromPeak = 0.0;
    apogeeCounter = 0;

    return;
  }

  dropFromPeak =
    peakAltitude -
    filteredAltitude;

  if (
    dropFromPeak >=
    apogeeDropThreshold
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
    apogeeConfirmSamples
  )
  {
    flightState = APOGEE;

    apogeeTime = millis();

    apogeeCounter = 0;

    Serial.println();
    Serial.println("APOGEE DETECTED");

    Serial.print("PEAK ALTITUDE: ");
    Serial.print(peakAltitude, 2);
    Serial.println(" m");

    deployRecovery();
  }
}

// ============================================================
// APOGEE -> DESCENT
// ============================================================

void updateApogeeState()
{
  if (flightState != APOGEE)
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

    // NEW:
    // Record when actual DESCENT state begins.
    descentStartTime = millis();

    landingCounter = 0;

    Serial.println(
      "STATE -> DESCENT"
    );
  }
}

// ============================================================
// LANDING DETECTION V1.6.2
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

  // ----------------------------------------------------------
  // NEW:
  // Do not allow landing immediately after entering DESCENT.
  // ----------------------------------------------------------

  if (
    millis() -
    descentStartTime <
    MIN_DESCENT_TIME_FOR_LANDING
  )
  {
    landingCounter = 0;
    return;
  }

  // ----------------------------------------------------------
  // Check whether altitude has become stable.
  // ----------------------------------------------------------

  bool altitudeStable =
    altitudeChange <=
    landingAltitudeChangeMax;

  // ----------------------------------------------------------
  // Check whether acceleration is approximately stationary 1g.
  // ----------------------------------------------------------

  bool accelerationStable =
    (
      accelerationMagnitude >=
      landingGMin
    )
    &&
    (
      accelerationMagnitude <=
      landingGMax
    );

  // ----------------------------------------------------------
  // BOTH conditions must remain valid.
  // ----------------------------------------------------------

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

  // ----------------------------------------------------------
  // Landing confirmation
  // ----------------------------------------------------------

  if (
    landingCounter >=
    landingConfirmSamples
  )
  {
    flightState = LANDED;

    landingTime = millis();

    landingCounter = 0;

    beaconTimer = millis();
    beaconOn = false;

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerActive = false;

    Serial.println();
    Serial.println("=======================");
    Serial.println(" LANDING DETECTED");
    Serial.println(" STATE -> LANDED");
    Serial.println(" RECOVERY BEACON ACTIVE");
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
    Serial.println(
      "RESET BLOCKED: SYSTEM FAULT"
    );

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
  Serial.println("=======================");
  Serial.println(" FLIGHT COMPUTER RESET");
  Serial.println(" STATE -> IDLE");
  Serial.println(" RECOVERY -> STOWED");
  Serial.println("=======================");
}

// ============================================================
// MANUAL BENCH TEST
// ============================================================

void benchTestLaunch()
{
  if (
    OPERATING_MODE != BENCH_MODE
  )
  {
    Serial.println(
      "BENCH COMMAND DISABLED"
    );

    return;
  }

  if (
    flightState != ARMED
  )
  {
    Serial.println(
      "TEST_LAUNCH BLOCKED: ARM FIRST"
    );

    return;
  }

  flightState = ASCENT;

  launchTime = millis();

  peakAltitude =
    filteredAltitude;

  dropFromPeak = 0.0;

  Serial.println();
  Serial.println(
    "TEST_LAUNCH -> ASCENT"
  );
}

// ------------------------------------------------------------

void benchTestApogee()
{
  if (
    OPERATING_MODE != BENCH_MODE
  )
  {
    return;
  }

  if (
    flightState != ASCENT
  )
  {
    Serial.println(
      "TEST_APOGEE BLOCKED: NOT ASCENT"
    );

    return;
  }

  flightState = APOGEE;

  apogeeTime = millis();

  Serial.println();
  Serial.println(
    "TEST_APOGEE -> APOGEE"
  );

  deployRecovery();
}

// ------------------------------------------------------------

void benchTestLanding()
{
  if (
    OPERATING_MODE != BENCH_MODE
  )
  {
    return;
  }

  if (
    flightState != DESCENT
  )
  {
    Serial.println(
      "TEST_LANDING BLOCKED: NOT DESCENT"
    );

    return;
  }

  flightState = LANDED;

  landingTime = millis();

  beaconTimer = millis();
  beaconOn = false;

  Serial.println();
  Serial.println(
    "TEST_LANDING -> LANDED"
  );

  Serial.println(
    "RECOVERY BEACON ACTIVE"
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
    Serial.readStringUntil('\n');

  command.trim();
  command.toUpperCase();

  // ARM
  if (command == "ARM")
  {
    if (flightState != IDLE)
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

    launchCounter = 0;
    apogeeCounter = 0;
    landingCounter = 0;

    peakAltitude =
      filteredAltitude;

    startBuzzer(150);

    Serial.println();
    Serial.println("SYSTEM ARMED");

    return;
  }

  // DISARM
  if (command == "DISARM")
  {
    if (flightState == ARMED)
    {
      flightState = IDLE;

      Serial.println(
        "SYSTEM DISARMED"
      );
    }
    else
    {
      Serial.println(
        "DISARM BLOCKED: NOT ARMED"
      );
    }

    return;
  }

  // RESET
  if (command == "RESET")
  {
    resetFlightComputer();
    return;
  }

  // BENCH COMMANDS
  if (command == "TEST_LAUNCH")
  {
    benchTestLaunch();
    return;
  }

  if (command == "TEST_APOGEE")
  {
    benchTestApogee();
    return;
  }

  if (command == "TEST_LANDING")
  {
    benchTestLanding();
    return;
  }

  // STATUS
  if (command == "STATUS")
  {
    Serial.println();
    Serial.println(
      "===== SYSTEM STATUS ====="
    );

    Serial.print("STATE: ");
    Serial.println(getStateName());

    Serial.print("HEALTH: ");

    if (systemHealthy)
      Serial.println("OK");
    else
      Serial.println("FAULT");

    Serial.print("RECOVERY: ");

    if (recoveryDeployed)
      Serial.println("DEPLOYED");
    else
      Serial.println("STOWED");

    Serial.print("ALTITUDE: ");
    Serial.print(filteredAltitude, 2);
    Serial.println(" m");

    Serial.print("PEAK: ");
    Serial.print(peakAltitude, 2);
    Serial.println(" m");

    Serial.print("TOTAL G: ");
    Serial.println(accelerationMagnitude, 2);

    Serial.println(
      "========================="
    );

    return;
  }

  Serial.print(
    "UNKNOWN COMMAND: "
  );

  Serial.println(command);
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
  display.println(getStateName());

  display.print("ALT:");
  display.print(filteredAltitude, 2);
  display.println("m");

  display.print("PEAK:");
  display.print(peakAltitude, 2);
  display.println("m");

  display.print("G:");
  display.println(
    accelerationMagnitude,
    2
  );

  display.print("REC:");

  if (recoveryDeployed)
    display.println("DEPLOYED");
  else
    display.println("STOWED");

  if (flightState == LANDED)
  {
    display.println(
      "BEACON:ACTIVE"
    );
  }
  else if (
    flightState == ASCENT ||
    flightState == APOGEE ||
    flightState == DESCENT
  )
  {
    display.print("T+:");

    display.print(
      timeSinceLaunch /
      1000.0,
      1
    );

    display.println("s");
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
  Serial.print(filteredAltitude, 2);

  Serial.print("m | PEAK=");
  Serial.print(peakAltitude, 2);

  Serial.print("m | dALT=");
  Serial.print(altitudeChange, 3);

  Serial.print("m | G=");
  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.print(" | LAND_COUNT=");
  Serial.print(landingCounter);

  Serial.print("/");
  Serial.print(
    landingConfirmSamples
  );

  Serial.print(" | REC=");

  if (recoveryDeployed)
    Serial.println("DEPLOYED");
  else
    Serial.println("STOWED");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  initializeHardware();

  if (flightState == FAULT)
  {
    updateStatusLEDs();

    Serial.println();
    Serial.println(
      "*** SYSTEM FAULT ***"
    );

    Serial.println(
      "ARMING DISABLED"
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
    " FLIGHT COMPUTER V1.6.2"
  );

  Serial.println(
    " LANDING DETECTION OPTIMIZED"
  );

  Serial.println(
    " HAND BENCH MODE READY"
  );

  Serial.println(
    "================================"
  );

  Serial.println();
  Serial.println("COMMANDS:");

  Serial.println("ARM");
  Serial.println("DISARM");
  Serial.println("RESET");
  Serial.println("TEST_LAUNCH");
  Serial.println("TEST_APOGEE");
  Serial.println("TEST_LANDING");
  Serial.println("STATUS");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  updateStatusLEDs();

  if (flightState == FAULT)
  {
    processSerialCommands();

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
