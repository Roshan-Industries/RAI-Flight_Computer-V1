// ============================================================
// RAI FLIGHT COMPUTER V1
// Version: FC_V1_v1.1
//
// Hardware:
// ESP32
// BMP280
// MPU6050-compatible IMU
// SSD1306 OLED
// SG90 Servo
// Buzzer
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

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
  ARMED
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

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

// ============================================================
// OUTPUT CONTROL VARIABLES
// ============================================================

bool servoActive = false;
unsigned long servoStartTime = 0;

bool buzzerActive = false;
unsigned long buzzerStartTime = 0;

// ============================================================
// TIMING
// ============================================================

unsigned long lastSensorUpdate = 0;

const unsigned long SENSOR_INTERVAL = 200;

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


// ------------------------------------------------------------

int16_t imuRead16(byte reg)
{
  Wire.beginTransmission(IMU_ADDR);

  Wire.write(reg);

  Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDR, 2);

  if (Wire.available() >= 2)
  {
    int16_t value =
      (Wire.read() << 8) |
       Wire.read();

    return value;
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
  Serial.println("================================");
  Serial.println(" RAI FLIGHT COMPUTER V1");
  Serial.println("================================");
  Serial.println("INITIALIZING HARDWARE...");
  Serial.println();

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("[OK] I2C BUS");

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

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
  display.println("V1");
  display.println();
  display.println("INITIALIZING...");

  display.display();

  Serial.println("[OK] OLED");

  // ----------------------------------------------------------
  // BMP280
  // ----------------------------------------------------------

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

  // ----------------------------------------------------------
  // IMU COMMUNICATION CHECK
  // ----------------------------------------------------------

  Wire.beginTransmission(IMU_ADDR);

  byte imuError =
    Wire.endTransmission();

  if (imuError != 0)
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

  // ----------------------------------------------------------
  // SERVO
  // ----------------------------------------------------------

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

  // ----------------------------------------------------------
  // BUZZER
  // ----------------------------------------------------------

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  // Startup beep

  digitalWrite(
    BUZZER_PIN,
    HIGH
  );

  delay(150);

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  Serial.println("[OK] BUZZER GPIO25");

  Serial.println();
  Serial.println("HARDWARE INITIALIZATION COMPLETE");
}

// ============================================================
// SENSOR CALIBRATION
// ============================================================

void calibrateSensors()
{
  flightState = CALIBRATION;

  display.clearDisplay();

  display.setCursor(0, 0);

  display.println("RAI FLIGHT COMPUTER");
  display.println("V1");
  display.println();
  display.println("CALIBRATING...");
  display.println("KEEP STILL");

  display.display();

  Serial.println();
  Serial.println("==============================");
  Serial.println("CALIBRATION STARTED");
  Serial.println("KEEP BOARD STILL");
  Serial.println("==============================");

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

  Serial.println();
  Serial.println("CALIBRATION COMPLETE");

  Serial.print("BASE ALTITUDE: ");
  Serial.print(baseAltitude, 2);
  Serial.println(" m");

  flightState = IDLE;

  delay(300);
}

// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  // ----------------------------------------------------------
  // BMP280
  // ----------------------------------------------------------

  temperature =
    bmp.readTemperature();

  pressure =
    bmp.readPressure() / 100.0F;

  absoluteAltitude =
    bmp.readAltitude(1013.25);

  relativeAltitude =
    absoluteAltitude -
    baseAltitude;

  // ----------------------------------------------------------
  // IMU
  // ----------------------------------------------------------

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

  // NOTE:
  // Your IMU reports WHO_AM_I = 0x98.
  // We are currently using MPU6050-compatible scaling.
  //
  // Accelerometer:
  // +/-2g -> 16384 LSB/g
  //
  // Gyroscope:
  // +/-250 deg/s -> 131 LSB/(deg/s)

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
}

// ============================================================
// SERVO CONTROL
// ============================================================

void startServoTest()
{
  if (servoActive)
  {
    return;
  }

  Serial.println();
  Serial.println("SERVO DEPLOY TEST");

  deploymentServo.write(
    SERVO_DEPLOYED
  );

  servoActive = true;

  servoStartTime =
    millis();
}


// ------------------------------------------------------------

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
// BUZZER CONTROL
// ============================================================

void startBuzzerTest()
{
  if (buzzerActive)
  {
    return;
  }

  Serial.println();
  Serial.println("BUZZER TEST");

  digitalWrite(
    BUZZER_PIN,
    HIGH
  );

  buzzerActive = true;

  buzzerStartTime =
    millis();
}


// ------------------------------------------------------------

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
// SERIAL COMMAND PROCESSING
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

  // ----------------------------------------------------------
  // ARM
  // ----------------------------------------------------------

  if (command == "ARM")
  {
    if (flightState == IDLE)
    {
      flightState = ARMED;

      Serial.println();
      Serial.println("SYSTEM ARMED");

      // Confirmation beep

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );

      delay(80);

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      delay(80);

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );

      delay(80);

      digitalWrite(
        BUZZER_PIN,
        LOW
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // DISARM
  // ----------------------------------------------------------

  if (command == "DISARM")
  {
    flightState = IDLE;

    Serial.println();
    Serial.println("SYSTEM DISARMED");

    return;
  }

  // ----------------------------------------------------------
  // SERVO
  // ----------------------------------------------------------

  if (command == "SERVO")
  {
    startServoTest();

    return;
  }

  // ----------------------------------------------------------
  // BUZZER
  // ----------------------------------------------------------

  if (command == "BUZZER")
  {
    startBuzzerTest();

    return;
  }

  // ----------------------------------------------------------

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

  display.print("STATE: ");

  display.println(
    getStateName()
  );

  display.print("ALT: ");

  display.print(
    relativeAltitude,
    1
  );

  display.println(" m");

  display.print("TMP: ");

  display.print(
    temperature,
    1
  );

  display.println(" C");

  display.print("A:");

  display.print(accelX, 1);
  display.print(",");

  display.print(accelY, 1);
  display.print(",");

  display.println(accelZ, 1);

  display.print("G:");

  display.print(gyroX, 0);
  display.print(",");

  display.print(gyroY, 0);
  display.print(",");

  display.println(gyroZ, 0);

  display.display();
}

// ============================================================
// SERIAL DATA OUTPUT
// ============================================================

void printSensorData()
{
  Serial.println(
    "----------------------------"
  );

  Serial.print("STATE: ");
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

  Serial.println(" m");

  Serial.print(
    "PRESSURE: "
  );

  Serial.print(
    pressure,
    2
  );

  Serial.println(" hPa");

  Serial.print(
    "TEMPERATURE: "
  );

  Serial.print(
    temperature,
    2
  );

  Serial.println(" C");

  Serial.print(
    "ACC X/Y/Z: "
  );

  Serial.print(accelX, 2);
  Serial.print(" / ");

  Serial.print(accelY, 2);
  Serial.print(" / ");

  Serial.println(accelZ, 2);

  Serial.print(
    "GYRO X/Y/Z: "
  );

  Serial.print(gyroX, 2);
  Serial.print(" / ");

  Serial.print(gyroY, 2);
  Serial.print(" / ");

  Serial.println(gyroZ, 2);
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

  display.clearDisplay();

  display.setCursor(0, 0);

  display.println(
    "RAI FLIGHT COMPUTER"
  );

  display.println("V1");
  display.println();

  display.println(
    "SYSTEM READY"
  );

  display.display();

  Serial.println();
  Serial.println("============================");
  Serial.println(" RAI FLIGHT COMPUTER READY");
  Serial.println("============================");

  Serial.println();
  Serial.println("AVAILABLE COMMANDS:");
  Serial.println("ARM");
  Serial.println("DISARM");
  Serial.println("SERVO");
  Serial.println("BUZZER");

  delay(1000);
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // Commands
  processSerialCommands();

  // Non-blocking output management
  updateServo();
  updateBuzzer();

  // Sensor/display update
  if (
      millis() - lastSensorUpdate
      >= SENSOR_INTERVAL
     )
  {
    lastSensorUpdate =
      millis();

    readSensors();

    updateDisplay();

    printSensorData();
  }
}
