// ============================================================
// ROSHAN AEROSPACE INDUSTRIES
// FLIGHT COMPUTER V1
//
// Firmware: FC_V1_v1.6_BenchValidated
//
// Hardware:
// ESP32
// BMP280
// MPU6050-compatible IMU
// SSD1306 OLED
// Servo
// Active Buzzer
//
// FINAL V1 BENCH DEVELOPMENT BASELINE
//
// Features:
// - Startup self-test
// - Sensor calibration
// - Launch detection
// - Apogee detection
// - One-time recovery deployment
// - Descent detection
// - Landing detection
// - Recovery beacon
// - Manual bench-event injection
// - State transition protection
// - Recovery deployment latch
// - Bench / Operational configuration framework
//
// IMPORTANT:
// BENCH profile is NOT flight-qualified.
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

// ------------------------------------------------------------
// CURRENT DEVELOPMENT MODE
// ------------------------------------------------------------

const OperatingMode OPERATING_MODE = BENCH_MODE;

// ============================================================
// PIN CONFIGURATION
// ============================================================

#define SDA_PIN       21
#define SCL_PIN       22

#define SERVO_PIN     18
#define BUZZER_PIN    25

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

const float BENCH_LANDING_ALT_CHANGE_MAX = 0.05;

const float BENCH_LANDING_G_MIN = 0.85;

const float BENCH_LANDING_G_MAX = 1.15;

const int BENCH_LANDING_CONFIRM_SAMPLES = 15;

// ============================================================
// OPERATIONAL PROFILE PLACEHOLDERS
// ============================================================
//
// DO NOT treat these as flight-qualified values.
//
// They exist so V1.6 has the correct software architecture.
// Actual values must come from vehicle testing.
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
// ACTIVE PROFILE VARIABLES
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
// TIMING
// ============================================================

const unsigned long SENSOR_INTERVAL = 100;

// ============================================================
// FILTERING
// ============================================================

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
// SENSOR HEALTH
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

float baseAltitude = 0.0;

float altitudeChange = 0.0;

// ------------------------------------------------------------
// IMU
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Beacon
// ------------------------------------------------------------

bool beaconOn = false;

unsigned long beaconTimer = 0;

// ============================================================
// SENSOR LOOP
// ============================================================

unsigned long lastSensorUpdate = 0;

// ============================================================
// IMU FUNCTIONS
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
    case BOOT:
      return "BOOT";

    case SELF_TEST:
      return "SELFTEST";

    case CALIBRATION:
      return "CAL";

    case IDLE:
      return "IDLE";

    case ARMED:
      return "ARMED";

    case ASCENT:
      return "ASCENT";

    case APOGEE:
      return "APOGEE";

    case DESCENT:
      return "DESCENT";

    case LANDED:
      return "LANDED";

    case FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

// ============================================================
// OPERATING PROFILE
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

    Serial.println(
      "PROFILE: BENCH TEST"
    );
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

    Serial.println(
      "PROFILE: OPERATIONAL"
    );

    Serial.println(
      "WARNING: OP VALUES NOT FLIGHT QUALIFIED"
    );
  }
}

// ============================================================
// BUZZER
// ============================================================

void startBuzzer(
  unsigned long duration
)
{
  digitalWrite(
    BUZZER_PIN,
    HIGH
  );

  buzzerActive = true;

  buzzerStartTime =
    millis();

  buzzerDuration =
    duration;
}

void updateBuzzer()
{
  if (
    buzzerActive &&
    millis() -
    buzzerStartTime >=
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
  if (
    flightState !=
    LANDED
  )
  {
    return;
  }

  unsigned long now =
    millis();

  if (!beaconOn)
  {
    if (
      now -
      beaconTimer >=
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
      now -
      beaconTimer >=
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
// STARTUP SELF TEST
// ============================================================

bool performSelfTest()
{
  flightState =
    SELF_TEST;

  Serial.println();
  Serial.println("======================");
  Serial.println(" STARTUP SELF TEST");
  Serial.println("======================");

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  oledHealthy =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    );

  if (oledHealthy)
  {
    Serial.println(
      "[PASS] OLED"
    );
  }

  else
  {
    Serial.println(
      "[FAIL] OLED"
    );
  }

  // ----------------------------------------------------------
  // BMP280
  // ----------------------------------------------------------

  bmpHealthy = false;

  if (bmp.begin(0x76))
  {
    bmpHealthy = true;

    Serial.println(
      "[PASS] BMP280 @ 0x76"
    );
  }

  else if (
    bmp.begin(0x77)
  )
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

  // ----------------------------------------------------------
  // IMU
  // ----------------------------------------------------------

  Wire.beginTransmission(
    IMU_ADDR
  );

  imuHealthy =
    (
      Wire.endTransmission()
      == 0
    );

  if (imuHealthy)
  {
    Serial.println(
      "[PASS] IMU"
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

  // ----------------------------------------------------------
  // RESULT
  // ----------------------------------------------------------

  systemHealthy =
    oledHealthy &&
    bmpHealthy &&
    imuHealthy;

  if (systemHealthy)
  {
    Serial.println();
    Serial.println(
      "SELF TEST: PASS"
    );

    return true;
  }

  Serial.println();
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
  flightState =
    BOOT;

  Serial.println();
  Serial.println("================================");
  Serial.println(" ROSHAN AEROSPACE INDUSTRIES");
  Serial.println(" FLIGHT COMPUTER V1");
  Serial.println(" Firmware V1.6");
  Serial.println("================================");

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  // Servo
  deploymentServo.setPeriodHertz(
    50
  );

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

  loadOperatingProfile();

  if (!performSelfTest())
  {
    flightState =
      FAULT;

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
    "V1.6"
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
    flightState =
      FAULT;

    return;
  }

  flightState =
    CALIBRATION;

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
  Serial.println(
    "CALIBRATION START"
  );

  const int samples =
    100;

  float altitudeSum =
    0.0;

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

  filteredAltitude =
    0.0;

  previousFilteredAltitude =
    0.0;

  flightState =
    IDLE;

  Serial.println(
    "CALIBRATION COMPLETE"
  );

  Serial.println(
    "STATE -> IDLE"
  );
}

// ============================================================
// SENSOR READ
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
      (
        1.0 -
        ALTITUDE_FILTER_ALPHA
      )
      *
      relativeAltitude
    );

  altitudeChange =
    fabs(
      filteredAltitude -
      previousFilteredAltitude
    );

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
  // ----------------------------------------------------------
  // HARD SOFTWARE LATCH
  // ----------------------------------------------------------

  if (recoveryDeployed)
  {
    Serial.println(
      "DEPLOY BLOCKED: ALREADY DEPLOYED"
    );

    return;
  }

  // Recovery deployment is only valid
  // during APOGEE/DESCENT in normal logic.

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

  recoveryDeployed =
    true;

  deploymentServo.write(
    SERVO_DEPLOYED
  );

  startBuzzer(300);

  Serial.println();
  Serial.println("=========================");
  Serial.println(" RECOVERY DEPLOYED");
  Serial.println(" DEPLOYMENT LATCH: SET");
  Serial.println("=========================");
}

// ============================================================
// LAUNCH DETECTION
// ============================================================

void detectLaunch()
{
  if (
    flightState !=
    ARMED
  )
  {
    launchCounter =
      0;

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
    launchCounter =
      0;
  }

  if (
    launchCounter >=
    launchConfirmSamples
  )
  {
    flightState =
      ASCENT;

    launchTime =
      millis();

    timeSinceLaunch =
      0;

    peakAltitude =
      filteredAltitude;

    dropFromPeak =
      0.0;

    launchCounter =
      0;

    apogeeCounter =
      0;

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
  if (
    flightState !=
    ASCENT
  )
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

    dropFromPeak =
      0.0;

    apogeeCounter =
      0;

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
    apogeeCounter =
      0;
  }

  if (
    apogeeCounter >=
    apogeeConfirmSamples
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
      "PEAK ALTITUDE: "
    );

    Serial.print(
      peakAltitude,
      2
    );

    Serial.println(
      " m"
    );

    deployRecovery();
  }
}

// ============================================================
// APOGEE -> DESCENT
// ============================================================

void updateApogeeState()
{
  if (
    flightState ==
    APOGEE
  )
  {
    if (
      millis() -
      apogeeTime >=
      500
    )
    {
      flightState =
        DESCENT;

      landingCounter =
        0;

      Serial.println(
        "STATE -> DESCENT"
      );
    }
  }
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
    landingCounter =
      0;

    return;
  }

  bool altitudeStable =
    altitudeChange <=
    landingAltitudeChangeMax;

  bool accelerationStable =
    accelerationMagnitude >=
    landingGMin
    &&
    accelerationMagnitude <=
    landingGMax;

  if (
    altitudeStable &&
    accelerationStable
  )
  {
    landingCounter++;
  }

  else
  {
    landingCounter =
      0;
  }

  if (
    landingCounter >=
    landingConfirmSamples
  )
  {
    flightState =
      LANDED;

    landingTime =
      millis();

    landingCounter =
      0;

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
  // Do not reset a hardware fault through
  // flight-state logic.

  if (
    flightState ==
    FAULT
  )
  {
    Serial.println(
      "RESET BLOCKED: SYSTEM FAULT"
    );

    return;
  }

  flightState =
    IDLE;

  launchCounter =
    0;

  apogeeCounter =
    0;

  landingCounter =
    0;

  launchTime =
    0;

  apogeeTime =
    0;

  landingTime =
    0;

  timeSinceLaunch =
    0;

  peakAltitude =
    filteredAltitude;

  dropFromPeak =
    0.0;

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
  Serial.println("=======================");
  Serial.println(" FLIGHT COMPUTER RESET");
  Serial.println(" STATE -> IDLE");
  Serial.println(" RECOVERY -> STOWED");
  Serial.println("=======================");
}

// ============================================================
// MANUAL BENCH TEST FUNCTIONS
// ============================================================

void benchTestLaunch()
{
  if (
    OPERATING_MODE !=
    BENCH_MODE
  )
  {
    Serial.println(
      "TEST COMMAND DISABLED"
    );

    return;
  }

  if (
    flightState !=
    ARMED
  )
  {
    Serial.println(
      "TEST_LAUNCH BLOCKED: ARM FIRST"
    );

    return;
  }

  flightState =
    ASCENT;

  launchTime =
    millis();

  peakAltitude =
    filteredAltitude;

  Serial.println(
    "TEST_LAUNCH -> ASCENT"
  );
}

// ------------------------------------------------------------

void benchTestApogee()
{
  if (
    OPERATING_MODE !=
    BENCH_MODE
  )
  {
    return;
  }

  if (
    flightState !=
    ASCENT
  )
  {
    Serial.println(
      "TEST_APOGEE BLOCKED: NOT ASCENT"
    );

    return;
  }

  flightState =
    APOGEE;

  apogeeTime =
    millis();

  Serial.println(
    "TEST_APOGEE -> APOGEE"
  );

  deployRecovery();
}

// ------------------------------------------------------------

void benchTestLanding()
{
  if (
    OPERATING_MODE !=
    BENCH_MODE
  )
  {
    return;
  }

  if (
    flightState !=
    DESCENT
  )
  {
    Serial.println(
      "TEST_LANDING BLOCKED: NOT DESCENT"
    );

    return;
  }

  flightState =
    LANDED;

  landingTime =
    millis();

  beaconTimer =
    millis();

  beaconOn =
    false;

  Serial.println(
    "TEST_LANDING -> LANDED"
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

  // ----------------------------------------------------------
  // ARM
  // ----------------------------------------------------------

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

    flightState =
      ARMED;

    launchCounter =
      0;

    apogeeCounter =
      0;

    landingCounter =
      0;

    peakAltitude =
      filteredAltitude;

    startBuzzer(150);

    Serial.println();
    Serial.println(
      "SYSTEM ARMED"
    );

    return;
  }

  // ----------------------------------------------------------
  // DISARM
  // ----------------------------------------------------------

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

  // ----------------------------------------------------------
  // RESET
  // ----------------------------------------------------------

  if (
    command ==
    "RESET"
  )
  {
    resetFlightComputer();

    return;
  }

  // ----------------------------------------------------------
  // BENCH TEST COMMANDS
  // ----------------------------------------------------------

  if (
    command ==
    "TEST_LAUNCH"
  )
  {
    benchTestLaunch();

    return;
  }

  if (
    command ==
    "TEST_APOGEE"
  )
  {
    benchTestApogee();

    return;
  }

  if (
    command ==
    "TEST_LANDING"
  )
  {
    benchTestLanding();

    return;
  }

  // ----------------------------------------------------------
  // STATUS
  // ----------------------------------------------------------

  if (
    command ==
    "STATUS"
  )
  {
    Serial.println();
    Serial.println("===== SYSTEM STATUS =====");

    Serial.print("STATE: ");
    Serial.println(getStateName());

    Serial.print("SYSTEM HEALTH: ");

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

    Serial.println("=========================");

    return;
  }

  Serial.print(
    "UNKNOWN COMMAND: "
  );

  Serial.println(
    command
  );
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

  if (
    recoveryDeployed
  )
  {
    display.println(
      "DEPLOYED"
    );
  }

  else
  {
    display.println(
      "STOWED"
    );
  }

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
// SERIAL TELEMETRY
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
    "m | PEAK="
  );

  Serial.print(
    peakAltitude,
    2
  );

  Serial.print(
    "m | dALT="
  );

  Serial.print(
    altitudeChange,
    3
  );

  Serial.print(
    "m | G="
  );

  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.print(
    " | REC="
  );

  if (
    recoveryDeployed
  )
  {
    Serial.println(
      "DEPLOYED"
    );
  }

  else
  {
    Serial.println(
      "STOWED"
    );
  }
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

  Serial.println();
  Serial.println("================================");
  Serial.println(" FLIGHT COMPUTER V1.6");
  Serial.println(" BENCH VALIDATION READY");
  Serial.println("================================");

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
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Fault mode
  // ----------------------------------------------------------

  if (
    flightState ==
    FAULT
  )
  {
    processSerialCommands();

    delay(100);

    return;
  }

  // ----------------------------------------------------------
  // Commands
  // ----------------------------------------------------------

  processSerialCommands();

  // ----------------------------------------------------------
  // Buzzer
  // ----------------------------------------------------------

  updateBuzzer();

  updateRecoveryBeacon();

  // ----------------------------------------------------------
  // Sensor / Flight Logic
  // ----------------------------------------------------------

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
      flightState ==
      ASCENT ||
      flightState ==
      APOGEE ||
      flightState ==
      DESCENT ||
      flightState ==
      LANDED
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
