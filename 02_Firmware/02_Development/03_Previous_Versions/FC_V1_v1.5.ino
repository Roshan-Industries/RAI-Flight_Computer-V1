// ============================================================
// RAI FLIGHT COMPUTER V1
// Version: FC_V1_v1.5_HAND_BENCH
//
// ESP32 + BMP280 + IMU + OLED + Servo + Buzzer
//
// NEW IN V1.5:
// - Landing detection
// - LANDED state
// - Recovery beacon buzzer
// - TEST_LANDING command
//
// IMPORTANT:
// Bench thresholds are for hand testing only.
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================
// MODE
// ============================================================

#define BENCH_TEST_MODE true

// ============================================================
// PIN CONFIGURATION
// ============================================================

#define SDA_PIN       21
#define SCL_PIN       22
#define SERVO_PIN     18
#define BUZZER_PIN    25

#define IMU_ADDR      0x68

// ============================================================
// OLED CONFIGURATION
// ============================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

// ============================================================
// SERVO CONFIGURATION
// ============================================================

#define SERVO_STOWED    0
#define SERVO_DEPLOYED  90

// ============================================================
// HAND BENCH THRESHOLDS
// ============================================================

// Launch
const float LAUNCH_G_THRESHOLD = 1.15;
const int LAUNCH_CONFIRM_SAMPLES = 1;

// Apogee
const float APOGEE_DROP_THRESHOLD = 0.08;
const int APOGEE_CONFIRM_SAMPLES = 2;
const unsigned long MIN_ASCENT_TIME = 300;

// Landing
const float LANDING_ALT_CHANGE_MAX = 0.05;   // meters
const float LANDING_G_MIN = 0.85;
const float LANDING_G_MAX = 1.15;
const int LANDING_CONFIRM_SAMPLES = 15;

// Loop timing
const unsigned long SENSOR_INTERVAL = 100;

// Altitude low-pass filter
const float ALTITUDE_FILTER_ALPHA = 0.80;

// Recovery beacon
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
  CALIBRATION,
  IDLE,
  ARMED,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED
};

FlightState flightState = BOOT;

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
// FLIGHT VARIABLES
// ============================================================

int launchCounter = 0;
int apogeeCounter = 0;
int landingCounter = 0;

float peakAltitude = 0.0;
float dropFromPeak = 0.0;
float altitudeChange = 0.0;

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

// Recovery beacon
bool beaconOn = false;
unsigned long beaconTimer = 0;

// ============================================================
// TIMING
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
    case CALIBRATION: return "CAL";
    case IDLE:        return "IDLE";
    case ARMED:       return "ARMED";
    case ASCENT:      return "ASCENT";
    case APOGEE:      return "APOGEE";
    case DESCENT:     return "DESCENT";
    case LANDED:      return "LANDED";
    default:          return "UNKNOWN";
  }
}

// ============================================================
// BUZZER CONTROL
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
    millis() - buzzerStartTime >= buzzerDuration
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
    if (now - beaconTimer >= BEACON_OFF_TIME)
    {
      digitalWrite(BUZZER_PIN, HIGH);

      beaconOn = true;
      beaconTimer = now;
    }
  }
  else
  {
    if (now - beaconTimer >= BEACON_ON_TIME)
    {
      digitalWrite(BUZZER_PIN, LOW);

      beaconOn = false;
      beaconTimer = now;
    }
  }
}

// ============================================================
// HARDWARE INITIALIZATION
// ============================================================

void initializeHardware()
{
  flightState = BOOT;

  Serial.println();
  Serial.println("=================================");
  Serial.println(" RAI FLIGHT COMPUTER V1.5");
  Serial.println(" HAND BENCH TEST MODE");
  Serial.println("=================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED
  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDR))
  {
    Serial.println("[ERROR] OLED");

    while (true)
    {
      delay(100);
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println("RAI FLIGHT COMPUTER");
  display.println("V1.5");
  display.println();
  display.println("INITIALIZING...");

  display.display();

  // BMP280
  bool bmpFound = false;

  if (bmp.begin(0x76))
  {
    bmpFound = true;
  }
  else if (bmp.begin(0x77))
  {
    bmpFound = true;
  }

  if (!bmpFound)
  {
    Serial.println("[ERROR] BMP280");

    while (true)
    {
      delay(100);
    }
  }

  // IMU
  Wire.beginTransmission(IMU_ADDR);

  if (Wire.endTransmission() != 0)
  {
    Serial.println("[ERROR] IMU");

    while (true)
    {
      delay(100);
    }
  }

  imuWrite(0x6B, 0x00);

  delay(100);

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
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  startBuzzer(150);

  Serial.println("[OK] HARDWARE");
}

// ============================================================
// CALIBRATION
// ============================================================

void calibrateSensors()
{
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

  for (int i = 0; i < samples; i++)
  {
    altitudeSum +=
      bmp.readAltitude(1013.25);

    gxSum +=
      imuRead16(0x43);

    gySum +=
      imuRead16(0x45);

    gzSum +=
      imuRead16(0x47);

    delay(20);
  }

  baseAltitude =
    altitudeSum / samples;

  gyroOffsetX =
    gxSum / (float)samples;

  gyroOffsetY =
    gySum / (float)samples;

  gyroOffsetZ =
    gzSum / (float)samples;

  filteredAltitude = 0.0;
  previousFilteredAltitude = 0.0;

  flightState = IDLE;

  Serial.println("CALIBRATION COMPLETE");
}

// ============================================================
// SENSOR READING
// ============================================================

void readSensors()
{
  temperature =
    bmp.readTemperature();

  pressure =
    bmp.readPressure() / 100.0F;

  absoluteAltitude =
    bmp.readAltitude(1013.25);

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

  // ---------------- IMU ----------------

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
    axRaw / 16384.0;

  accelY =
    ayRaw / 16384.0;

  accelZ =
    azRaw / 16384.0;

  gyroX =
    (gxRaw - gyroOffsetX) / 131.0;

  gyroY =
    (gyRaw - gyroOffsetY) / 131.0;

  gyroZ =
    (gzRaw - gyroOffsetZ) / 131.0;

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

  recoveryDeployed = true;

  deploymentServo.write(
    SERVO_DEPLOYED
  );

  startBuzzer(300);

  Serial.println();
  Serial.println("=========================");
  Serial.println(" RECOVERY DEPLOYED");
  Serial.println("=========================");
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

  if (
    accelerationMagnitude >=
    LAUNCH_G_THRESHOLD
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
    LAUNCH_CONFIRM_SAMPLES
  )
  {
    flightState = ASCENT;

    launchTime = millis();

    timeSinceLaunch = 0;

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
// APOGEE DETECTION
// ============================================================

void detectApogee()
{
  if (flightState != ASCENT)
  {
    return;
  }

  timeSinceLaunch =
    millis() - launchTime;

  if (
    timeSinceLaunch <
    MIN_ASCENT_TIME
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
    APOGEE_DROP_THRESHOLD
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
    flightState = APOGEE;

    apogeeTime = millis();

    apogeeCounter = 0;

    Serial.println();
    Serial.println("APOGEE DETECTED");

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

      landingCounter = 0;

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
    landingCounter = 0;
    return;
  }

  bool altitudeStable =
    altitudeChange <=
    LANDING_ALT_CHANGE_MAX;

  bool accelerationStable =
    accelerationMagnitude >= LANDING_G_MIN &&
    accelerationMagnitude <= LANDING_G_MAX;

  if (
    altitudeStable &&
    accelerationStable
  )
  {
    landingCounter++;

    Serial.print(
      "LANDING CHECK "
    );

    Serial.print(
      landingCounter
    );

    Serial.print("/");
    Serial.println(
      LANDING_CONFIRM_SAMPLES
    );
  }
  else
  {
    landingCounter = 0;
  }

  if (
    landingCounter >=
    LANDING_CONFIRM_SAMPLES
  )
  {
    flightState = LANDED;

    landingTime = millis();

    landingCounter = 0;

    // Start beacon timer
    beaconTimer = millis();
    beaconOn = false;

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    Serial.println();
    Serial.println("========================");
    Serial.println(" LANDING DETECTED");
    Serial.println(" STATE -> LANDED");
    Serial.println(" RECOVERY BEACON ACTIVE");
    Serial.println("========================");
  }
}

// ============================================================
// MANUAL BENCH TESTS
// ============================================================

void testLaunch()
{
  if (!BENCH_TEST_MODE)
  {
    return;
  }

  flightState = ASCENT;

  launchTime = millis();

  timeSinceLaunch = 0;

  peakAltitude =
    filteredAltitude;

  Serial.println(
    "TEST_LAUNCH -> ASCENT"
  );
}

void testApogee()
{
  if (!BENCH_TEST_MODE)
  {
    return;
  }

  if (
    flightState !=
    ASCENT
  )
  {
    Serial.println(
      "TEST_APOGEE REQUIRES ASCENT"
    );

    return;
  }

  flightState = APOGEE;

  apogeeTime =
    millis();

  Serial.println(
    "TEST_APOGEE -> APOGEE"
  );

  deployRecovery();
}

void testLanding()
{
  if (!BENCH_TEST_MODE)
  {
    return;
  }

  flightState = LANDED;

  landingTime =
    millis();

  beaconTimer =
    millis();

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
// RESET
// ============================================================

void resetFlightComputer()
{
  flightState = IDLE;

  launchCounter = 0;
  apogeeCounter = 0;
  landingCounter = 0;

  launchTime = 0;
  apogeeTime = 0;
  landingTime = 0;
  timeSinceLaunch = 0;

  peakAltitude = 0.0;
  dropFromPeak = 0.0;

  filteredAltitude =
    relativeAltitude;

  previousFilteredAltitude =
    filteredAltitude;

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
  Serial.println("==========================");
  Serial.println(" FLIGHT COMPUTER RESET");
  Serial.println(" STATE -> IDLE");
  Serial.println(" RECOVERY -> STOWED");
  Serial.println("==========================");
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

  if (
    command == "ARM"
  )
  {
    if (
      flightState ==
      IDLE
    )
    {
      flightState =
        ARMED;

      launchCounter = 0;
      apogeeCounter = 0;
      landingCounter = 0;

      peakAltitude =
        filteredAltitude;

      startBuzzer(150);

      Serial.println(
        "SYSTEM ARMED"
      );
    }

    return;
  }

  if (
    command ==
    "DISARM"
  )
  {
    flightState =
      IDLE;

    Serial.println(
      "SYSTEM DISARMED"
    );

    return;
  }

  if (
    command ==
    "RESET"
  )
  {
    resetFlightComputer();

    return;
  }

  if (
    command ==
    "TEST_LAUNCH"
  )
  {
    testLaunch();

    return;
  }

  if (
    command ==
    "TEST_APOGEE"
  )
  {
    testApogee();

    return;
  }

  if (
    command ==
    "TEST_LANDING"
  )
  {
    testLanding();

    return;
  }

  if (
    command ==
    "SERVO_DEPLOY"
  )
  {
    deployRecovery();

    return;
  }

  if (
    command ==
    "BUZZER"
  )
  {
    startBuzzer(500);

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
// OLED DISPLAY
// ============================================================

void updateDisplay()
{
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
  else if (
    flightState ==
    ASCENT ||
    flightState ==
    APOGEE ||
    flightState ==
    DESCENT
  )
  {
    display.print(
      "T+:"
    );

    display.print(
      timeSinceLaunch /
      1000.0,
      1
    );

    display.println(
      "s"
    );
  }

  display.display();
}

// ============================================================
// SERIAL TELEMETRY
// ============================================================

void printSensorData()
{
  Serial.println(
    "----------------------------"
  );

  Serial.print(
    "STATE: "
  );

  Serial.println(
    getStateName()
  );

  Serial.print(
    "ALT FILTERED: "
  );

  Serial.print(
    filteredAltitude,
    2
  );

  Serial.println(
    " m"
  );

  Serial.print(
    "ALT CHANGE: "
  );

  Serial.print(
    altitudeChange,
    3
  );

  Serial.println(
    " m"
  );

  Serial.print(
    "TOTAL G: "
  );

  Serial.println(
    accelerationMagnitude,
    2
  );

  Serial.print(
    "RECOVERY: "
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

  calibrateSensors();

  Serial.println();
  Serial.println("============================");
  Serial.println(" FLIGHT COMPUTER V1.5");
  Serial.println(" HAND BENCH MODE READY");
  Serial.println("============================");

  Serial.println();
  Serial.println("COMMANDS:");

  Serial.println("ARM");
  Serial.println("DISARM");
  Serial.println("RESET");

  Serial.println("TEST_LAUNCH");
  Serial.println("TEST_APOGEE");
  Serial.println("TEST_LANDING");

  Serial.println("SERVO_DEPLOY");
  Serial.println("BUZZER");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
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

    printSensorData();
  }
}
