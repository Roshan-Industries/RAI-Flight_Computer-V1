// ============================================================
// RAI FLIGHT COMPUTER V1
// Version: FC_V1_v1.4b_HAND_BENCH
//
// ESP32 + BMP280 + IMU + OLED + Servo + Buzzer
//
// PURPOSE:
// - Easy hand-based bench testing
// - Automatic launch detection
// - Automatic apogee detection
// - Automatic one-time recovery deployment
// - Manual event injection still available
//
// IMPORTANT:
// These thresholds are NOT flight-qualified.
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
// HAND BENCH TEST THRESHOLDS
// ============================================================

// Easy enough to trigger by hand
const float LAUNCH_G_THRESHOLD = 1.15;

// One strong sample is enough in bench mode
const int LAUNCH_CONFIRM_SAMPLES = 1;

// Very small altitude drop for hand testing
const float APOGEE_DROP_THRESHOLD = 0.08;

// Still require two readings to reduce noise
const int APOGEE_CONFIRM_SAMPLES = 2;

// Short wait after launch
const unsigned long MIN_ASCENT_TIME = 300;

// Main sensor update rate
const unsigned long SENSOR_INTERVAL = 100;

// ============================================================
// ALTITUDE FILTER
// ============================================================

// Larger alpha = smoother/slower response
const float ALTITUDE_FILTER_ALPHA = 0.80;

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
  DESCENT
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

float peakAltitude = 0.0;
float dropFromPeak = 0.0;

unsigned long launchTime = 0;
unsigned long apogeeTime = 0;
unsigned long timeSinceLaunch = 0;

// ============================================================
// RECOVERY VARIABLES
// ============================================================

bool recoveryDeployed = false;

// ============================================================
// BUZZER VARIABLES
// ============================================================

bool buzzerActive = false;

unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;

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
    case BOOT:
      return "BOOT";

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

    default:
      return "UNKNOWN";
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
// HARDWARE INITIALIZATION
// ============================================================

void initializeHardware()
{
  flightState = BOOT;

  Serial.println();
  Serial.println("=================================");
  Serial.println(" RAI FLIGHT COMPUTER V1.4b");
  Serial.println(" HAND BENCH TEST MODE");
  Serial.println("=================================");

  // ---------------- I2C ----------------

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("[OK] I2C");

  // ---------------- OLED ----------------

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
  display.println("V1.4b");
  display.println();
  display.println("INITIALIZING...");

  display.display();

  Serial.println("[OK] OLED");

  // ---------------- BMP280 ----------------

  bool bmpFound = false;

  if (bmp.begin(0x76))
  {
    bmpFound = true;
    Serial.println("[OK] BMP280 @ 0x76");
  }

  else if (bmp.begin(0x77))
  {
    bmpFound = true;
    Serial.println("[OK] BMP280 @ 0x77");
  }

  if (!bmpFound)
  {
    Serial.println("[ERROR] BMP280");

    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("SYSTEM ERROR");
    display.println();
    display.println("BMP280 FAILED");

    display.display();

    while (true)
    {
      delay(100);
    }
  }

  // ---------------- IMU ----------------

  Wire.beginTransmission(IMU_ADDR);

  if (Wire.endTransmission() != 0)
  {
    Serial.println("[ERROR] IMU");

    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("SYSTEM ERROR");
    display.println();
    display.println("IMU FAILED");

    display.display();

    while (true)
    {
      delay(100);
    }
  }

  imuWrite(0x6B, 0x00);

  delay(100);

  Serial.println("[OK] IMU @ 0x68");

  // ---------------- SERVO ----------------

  deploymentServo.setPeriodHertz(50);

  deploymentServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  deploymentServo.write(
    SERVO_STOWED
  );

  Serial.println("[OK] SERVO GPIO18");

  // ---------------- BUZZER ----------------

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  startBuzzer(150);

  Serial.println("[OK] BUZZER GPIO25");
}

// ============================================================
// CALIBRATION
// ============================================================

void calibrateSensors()
{
  flightState = CALIBRATION;

  display.clearDisplay();

  display.setCursor(0, 0);

  display.println("RAI FLIGHT COMPUTER");
  display.println();
  display.println("CALIBRATING...");
  display.println("KEEP STILL");

  display.display();

  Serial.println();
  Serial.println("CALIBRATION STARTED");
  Serial.println("KEEP BOARD STILL");

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

  // Initialize filter at zero-reference altitude
  filteredAltitude = 0.0;

  flightState = IDLE;

  Serial.println();
  Serial.println("CALIBRATION COMPLETE");

  Serial.print("BASE ALTITUDE: ");
  Serial.print(baseAltitude, 2);
  Serial.println(" m");
}

// ============================================================
// SENSOR READING
// ============================================================

void readSensors()
{
  // ---------------- BMP280 ----------------

  temperature =
    bmp.readTemperature();

  pressure =
    bmp.readPressure() / 100.0F;

  absoluteAltitude =
    bmp.readAltitude(1013.25);

  relativeAltitude =
    absoluteAltitude - baseAltitude;

  // Low-pass filter

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

  // MPU6050-compatible scaling assumption

  accelX =
    axRaw / 16384.0;

  accelY =
    ayRaw / 16384.0;

  accelZ =
    azRaw / 16384.0;

  gyroX =
    (gxRaw - gyroOffsetX)
    / 131.0;

  gyroY =
    (gyRaw - gyroOffsetY)
    / 131.0;

  gyroZ =
    (gzRaw - gyroOffsetZ)
    / 131.0;

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
  // Latching protection:
  // deployment can only happen once.

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
  Serial.println("==========================");
  Serial.println(" RECOVERY DEPLOYED");
  Serial.println(" SERVO -> 90 DEG");
  Serial.println("==========================");
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

    Serial.print("LAUNCH CHECK: G=");
    Serial.print(
      accelerationMagnitude,
      2
    );

    Serial.print("  COUNT=");
    Serial.println(
      launchCounter
    );
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
    Serial.println("====================");
    Serial.println(" LAUNCH DETECTED");
    Serial.println(" STATE -> ASCENT");
    Serial.println("====================");
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

  // Update peak altitude

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

  Serial.print("ALT RAW=");
  Serial.print(
    relativeAltitude,
    2
  );

  Serial.print(" FILT=");
  Serial.print(
    filteredAltitude,
    2
  );

  Serial.print(" PEAK=");
  Serial.print(
    peakAltitude,
    2
  );

  Serial.print(" DROP=");
  Serial.print(
    dropFromPeak,
    2
  );

  Serial.print(" COUNT=");
  Serial.println(
    apogeeCounter
  );

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

    apogeeTime =
      millis();

    apogeeCounter = 0;

    Serial.println();
    Serial.println("======================");
    Serial.println(" APOGEE DETECTED");

    Serial.print(" PEAK ALTITUDE: ");
    Serial.print(
      peakAltitude,
      2
    );
    Serial.println(" m");

    Serial.println("======================");

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

      Serial.println();
      Serial.println(
        "STATE -> DESCENT"
      );
    }
  }
}

// ============================================================
// MANUAL BENCH INJECTION
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

  dropFromPeak = 0.0;

  launchCounter = 0;
  apogeeCounter = 0;

  Serial.println();
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
    flightState != ASCENT
  )
  {
    Serial.println(
      "TEST_APOGEE REQUIRES ASCENT"
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

void testDescent()
{
  if (!BENCH_TEST_MODE)
  {
    return;
  }

  flightState = DESCENT;

  Serial.println();
  Serial.println(
    "TEST_DESCENT -> DESCENT"
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

  launchTime = 0;
  apogeeTime = 0;

  timeSinceLaunch = 0;

  peakAltitude = 0.0;
  dropFromPeak = 0.0;

  // Reinitialize filtered altitude to current relative altitude
  filteredAltitude =
    relativeAltitude;

  recoveryDeployed = false;

  deploymentServo.write(
    SERVO_STOWED
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  buzzerActive = false;

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

  // ---------------- ARM ----------------

  if (command == "ARM")
  {
    if (flightState == IDLE)
    {
      flightState = ARMED;

      launchCounter = 0;
      apogeeCounter = 0;

      // Set current filtered altitude as
      // starting reference for peak tracking.

      peakAltitude =
        filteredAltitude;

      startBuzzer(150);

      Serial.println();
      Serial.println("SYSTEM ARMED");

      Serial.println(
        "HAND TEST: GIVE QUICK UPWARD MOVEMENT"
      );
    }

    return;
  }

  // ---------------- DISARM ----------------

  if (command == "DISARM")
  {
    flightState = IDLE;

    launchCounter = 0;
    apogeeCounter = 0;

    Serial.println(
      "SYSTEM DISARMED"
    );

    return;
  }

  // ---------------- RESET ----------------

  if (command == "RESET")
  {
    resetFlightComputer();

    return;
  }

  // ---------------- MANUAL TESTING ----------------

  if (command == "TEST_LAUNCH")
  {
    testLaunch();

    return;
  }

  if (command == "TEST_APOGEE")
  {
    testApogee();

    return;
  }

  if (command == "TEST_DESCENT")
  {
    testDescent();

    return;
  }

  // ---------------- SERVO ----------------

  if (command == "SERVO_DEPLOY")
  {
    deployRecovery();

    return;
  }

  // ---------------- BUZZER ----------------

  if (command == "BUZZER")
  {
    startBuzzer(500);

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
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(0, 0);

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
    flightState == ASCENT ||
    flightState == APOGEE ||
    flightState == DESCENT
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
    "ALT RAW: "
  );

  Serial.print(
    relativeAltitude,
    2
  );

  Serial.println(
    " m"
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
    "PEAK: "
  );

  Serial.print(
    peakAltitude,
    2
  );

  Serial.println(
    " m"
  );

  Serial.print(
    "DROP: "
  );

  Serial.print(
    dropFromPeak,
    2
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
  Serial.begin(115200);

  delay(1000);

  initializeHardware();

  calibrateSensors();

  Serial.println();
  Serial.println("============================");
  Serial.println(" FLIGHT COMPUTER V1.4b");
  Serial.println(" HAND BENCH MODE READY");
  Serial.println("============================");

  Serial.println();
  Serial.println("COMMANDS:");
  Serial.println("ARM");
  Serial.println("DISARM");
  Serial.println("RESET");

  Serial.println("TEST_LAUNCH");
  Serial.println("TEST_APOGEE");
  Serial.println("TEST_DESCENT");

  Serial.println("SERVO_DEPLOY");
  Serial.println("BUZZER");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // Serial commands
  processSerialCommands();

  // Non-blocking buzzer
  updateBuzzer();

  // Sensor update
  if (
    millis() -
    lastSensorUpdate >=
    SENSOR_INTERVAL
  )
  {
    lastSensorUpdate =
      millis();

    readSensors();

    // Automatic sensor-based logic
    detectLaunch();

    detectApogee();

    updateApogeeState();

    if (
      flightState ==
      ASCENT ||
      flightState ==
      APOGEE ||
      flightState ==
      DESCENT
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
