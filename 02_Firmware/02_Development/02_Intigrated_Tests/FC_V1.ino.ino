#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

// ===============================
// OLED Configuration
// ===============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===============================
// Sensors
// ===============================
Adafruit_BMP280 bmp;
MPU6050 mpu;

// ===============================
// Servo
// ===============================
Servo parachuteServo;

// ===============================
// Pin Definitions
// ===============================
#define SERVO_PIN 17
#define BUZZER_PIN 25

// ===============================
// Flight States
// ===============================
enum FlightState
{
    STARTUP,
    IDLE,
    ARMED,
    ASCENT,
    DESCENT,
    LANDED,
    ERROR_STATE
};

FlightState currentState = STARTUP;

// ===============================
// Sensor Variables
// ===============================
float altitude = 0;
float temperature = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// ===============================
// Servo Positions
// ===============================
const int SERVO_LOCKED = 0;
const int SERVO_DEPLOY = 90;

// ===============================
// Function Prototypes
// ===============================
void initializeHardware();
void updateSensors();
void updateDisplay();
void updateState();
void updateServo();
void updateBuzzer();

// ===============================
// Setup
// ===============================
void setup()
{
    Serial.begin(115200);

    initializeHardware();

    currentState = IDLE;
}

// ===============================
// Main Loop
// ===============================
void loop()
{
    updateSensors();

    updateState();

    updateServo();

    updateBuzzer();

    updateDisplay();
}
