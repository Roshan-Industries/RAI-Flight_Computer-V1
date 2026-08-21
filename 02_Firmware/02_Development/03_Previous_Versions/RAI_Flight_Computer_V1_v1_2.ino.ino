// ============================================================
// RAI FLIGHT COMPUTER V1
// Version: FC_V1_v1.3_BENCH
//
// ESP32 + BMP280 + IMU + OLED + Servo + Buzzer
//
// BENCH TEST FEATURES:
// - Automatic launch detection
// - ASCENT state
// - Peak altitude tracking
// - Relaxed apogee detection
// - APOGEE state
// - DESCENT state
// - Servo does NOT auto-deploy yet
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <math.h>

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

const unsigned long SERVO_TEST_TIME = 1000;

// ============================================================
// BUZZER CONFIGURATION
// ============================================================

const unsigned long BUZZER_TEST_TIME = 500;

// ============================================================
// BENCH FLIGHT LOGIC SETTINGS
// ============================================================

// Launch detection
const float LAUNCH_G_THRESHOLD = 1.40;
const int LAUNCH_CONFIRM_SAMPLES = 3;

// BENCH-ONLY apogee detection
const float APOGEE_DROP_THRESHOLD = 0.20;
const int APOGEE_CONFIRM_SAMPLES = 2;
const unsigned long MIN_ASCENT_TIME = 500;

// Sensor loop
const unsigned long SENSOR_INTERVAL = 100;

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
// FLIGHT LOGIC VARIABLES
// ============================================================

int launchCounter = 0;
int apogeeCounter = 0;

float peakAltitude = 0.0;
float dropFromPeak = 0.0;

unsigned long launchTime = 0;
unsigned long apogeeTime = 0;
unsigned long timeSinceLaunch = 0;

// ============================================================
// OUTPUT VARIABLES
// ============================================================

bool servoActive = false;
unsigned long servoStartTime = 0;

bool buzzerActive = false;
unsigned long buzzerStartTime = 0;

// ============================================================
// TIMING
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
// HARDWARE INITIALIZATION
// ============================================================

void initializeHardware()
{
  flightState = BOOT;

  Serial.println();
  Serial.println("==============================");
  Serial.println(" RAI FLIGHT COMPUTER V1.3");
  Serial.println(" BENCH TEST VERSION");
  Serial.println("==============================");

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
  display.println("V1.3 BENCH");
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

  // Wake IMU
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

  deploymentServo.write(SERVO_STOWED);

  Serial.println("[OK] SERVO GPIO18");

  // ---------------- BUZZER ----------------

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  // Startup beep
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);

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

    gxSum += imuRead16(0x43);
    gySum += imuRead16(0x45);
    gzSum += imuRead16(0x47);

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

  flightState = IDLE;

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
    absoluteAltitude -
    baseAltitude;

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
// LAUNCH DETECTION
// ============================================================

void detectLaunch()
{
  if (flightState != ARMED)
  {
    launchCounter = 0;
    return;
  }

  if (accelerationMagnitude >= LAUNCH_G_THRESHOLD)
  {
    launchCounter++;

    Serial.print("LAUNCH CONFIRM: ");
    Serial.print(launchCounter);
    Serial.print("/");
    Serial.println(LAUNCH_CONFIRM_SAMPLES);
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

    peakAltitude = relativeAltitude;

    apogeeCounter = 0;

    launchCounter = 0;

    Serial.println();
    Serial.println("======================");
    Serial.println("    LAUNCH DETECTED");
    Serial.println("    STATE: ASCENT");
    Serial.println("======================");

    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
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

  // Ignore early noise
  if (
    timeSinceLaunch <
    MIN_ASCENT_TIME
  )
  {
    return;
  }

  // Update peak altitude
  if (
    relativeAltitude >
    peakAltitude
  )
  {
    peakAltitude =
      relativeAltitude;

    apogeeCounter = 0;

    return;
  }

  dropFromPeak =
    peakAltitude -
    relativeAltitude;

  // Debug information

  Serial.print("ALT=");
  Serial.print(
    relativeAltitude,
    2
  );

  Serial.print("  PEAK=");
  Serial.print(
    peakAltitude,
    2
  );

  Serial.print("  DROP=");
  Serial.print(
    dropFromPeak,
    2
  );

  Serial.print("  COUNT=");
  Serial.println(
    apogeeCounter
  );

  if (
    dropFromPeak >=
    APOGEE_DROP_THRESHOLD
  )
  {
    apogeeCounter++;

    Serial.print(
      "APOGEE CONFIRM: "
    );

    Serial.print(
      apogeeCounter
    );

    Serial.print("/");

    Serial.println(
      APOGEE_CONFIRM_SAMPLES
    );
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
    Serial.println("    APOGEE DETECTED");
    Serial.print("PEAK ALTITUDE: ");
    Serial.print(
      peakAltitude,
      2
    );
    Serial.println(" m");
    Serial.println("======================");

    // Buzzer only.
    // No automatic servo deployment yet.

    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
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
      millis() - apogeeTime
      >= 500
    )
    {
      flightState =
        DESCENT;

      Serial.println();
      Serial.println(
        "STATE: DESCENT"
      );
    }
  }
}

// ============================================================
// SERVO TEST
// ============================================================

void startServoTest()
{
  if (servoActive)
  {
    return;
  }

  deploymentServo.write(
    SERVO_DEPLOYED
  );

  servoActive = true;

  servoStartTime =
    millis();

  Serial.println(
    "SERVO DEPLOY TEST"
  );
}

void updateServo()
{
  if (
    servoActive &&
    millis() - servoStartTime
    >= SERVO_TEST_TIME
  )
  {
    deploymentServo.write(
      SERVO_STOWED
    );

    servoActive = false;

    Serial.println(
      "SERVO RETURNED"
    );
  }
}

// ============================================================
// BUZZER TEST
// ============================================================

void startBuzzerTest()
{
  if (buzzerActive)
  {
    return;
  }

  digitalWrite(
    BUZZER_PIN,
    HIGH
  );

  buzzerActive = true;

  buzzerStartTime =
    millis();

  Serial.println(
    "BUZZER TEST"
  );
}

void updateBuzzer()
{
  if (
    buzzerActive &&
    millis() - buzzerStartTime
    >= BUZZER_TEST_TIME
  )
  {
    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerActive = false;

    Serial.println(
      "BUZZER OFF"
    );
  }
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

  if (
    command == "ARM" &&
    flightState == IDLE
  )
  {
    flightState = ARMED;

    launchCounter = 0;
    apogeeCounter = 0;

    Serial.println();
    Serial.println(
      "SYSTEM ARMED"
    );

    return;
  }

  // ---------------- DISARM ----------------

  if (
    command ==
    "DISARM"
  )
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

  if (
    command ==
    "RESET"
  )
  {
    flightState = IDLE;

    launchCounter = 0;
    apogeeCounter = 0;

    launchTime = 0;
    apogeeTime = 0;
    timeSinceLaunch = 0;

    peakAltitude = 0.0;
    dropFromPeak = 0.0;

    deploymentServo.write(
      SERVO_STOWED
    );

    Serial.println(
      "FLIGHT LOGIC RESET"
    );

    return;
  }

  // ---------------- SERVO ----------------

  if (
    command ==
    "SERVO"
  )
  {
    startServoTest();

    return;
  }

  // ---------------- BUZZER ----------------

  if (
    command ==
    "BUZZER"
  )
  {
    startBuzzerTest();

    return;
  }

  Serial.print(
    "UNKNOWN COMMAND: "
  );

  Serial.println(command);
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
    relativeAltitude,
    1
  );

  display.println(
    "m"
  );

  display.print(
    "PEAK:"
  );

  display.print(
    peakAltitude,
    1
  );

  display.println(
    "m"
  );

  display.print(
    "DROP:"
  );

  display.print(
    dropFromPeak,
    1
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
// SERIAL OUTPUT
// ============================================================

void printSensorData()
{
  Serial.println(
    "-------------------------"
  );

  Serial.print(
    "STATE: "
  );

  Serial.println(
    getStateName()
  );

  Serial.print(
    "ALTITUDE: "
  );

  Serial.print(
    relativeAltitude,
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

  if (
    flightState == ASCENT ||
    flightState == APOGEE ||
    flightState == DESCENT
  )
  {
    Serial.print(
      "T+: "
    );

    Serial.print(
      timeSinceLaunch /
      1000.0,
      2
    );

    Serial.println(
      " s"
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
  Serial.println("=========================");
  Serial.println(" FLIGHT COMPUTER READY");
  Serial.println("=========================");

  Serial.println();
  Serial.println("COMMANDS:");
  Serial.println("ARM");
  Serial.println("DISARM");
  Serial.println("RESET");
  Serial.println("SERVO");
  Serial.println("BUZZER");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  processSerialCommands();

  updateServo();
  updateBuzzer();

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
