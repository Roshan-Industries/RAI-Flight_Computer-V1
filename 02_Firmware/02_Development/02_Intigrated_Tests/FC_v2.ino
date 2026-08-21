#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

#define IMU_ADDR 0x68

Adafruit_BMP280 bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Reference altitude
float baseAltitude = 0;

// Gyro offsets
float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

void imuWrite(byte reg, byte value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t imuRead16(byte reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDR, 2);

  if (Wire.available() >= 2) {
    return (int16_t)((Wire.read() << 8) | Wire.read());
  }

  return 0;
}

void calibrateSensors() {

  Serial.println("Calibration started - KEEP BOARD STILL");

  float altitudeSum = 0;
  long gxSum = 0;
  long gySum = 0;
  long gzSum = 0;

  const int samples = 100;

  for (int i = 0; i < samples; i++) {

    altitudeSum += bmp.readAltitude(1013.25);

    gxSum += imuRead16(0x43);
    gySum += imuRead16(0x45);
    gzSum += imuRead16(0x47);

    delay(20);
  }

  baseAltitude = altitudeSum / samples;

  gyroOffsetX = gxSum / (float)samples;
  gyroOffsetY = gySum / (float)samples;
  gyroOffsetZ = gzSum / (float)samples;

  Serial.println("Calibration complete.");
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED ERROR");
    while (1);
  }

  // BMP280
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 ERROR");
      while (1);
    }
  }

  // Wake IMU
  imuWrite(0x6B, 0x00);

  delay(500);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println("Flight Computer V1");
  display.println();
  display.println("CALIBRATING...");
  display.println("KEEP STILL");

  display.display();

  calibrateSensors();
}

void loop() {

  // ---------------- BMP280 ----------------

  float currentAltitude =
      bmp.readAltitude(1013.25);

  float relativeAltitude =
      currentAltitude - baseAltitude;

  float temperature =
      bmp.readTemperature();

  // ---------------- IMU ----------------

  int16_t axRaw = imuRead16(0x3B);
  int16_t ayRaw = imuRead16(0x3D);
  int16_t azRaw = imuRead16(0x3F);

  int16_t gxRaw = imuRead16(0x43);
  int16_t gyRaw = imuRead16(0x45);
  int16_t gzRaw = imuRead16(0x47);

  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float gx =
      (gxRaw - gyroOffsetX) / 131.0;

  float gy =
      (gyRaw - gyroOffsetY) / 131.0;

  float gz =
      (gzRaw - gyroOffsetZ) / 131.0;

  // ---------------- SERIAL ----------------

  Serial.println("------ FC SENSOR DATA ------");

  Serial.print("Relative Altitude: ");
  Serial.print(relativeAltitude, 2);
  Serial.println(" m");

  Serial.print("Acceleration: ");
  Serial.print(ax, 2);
  Serial.print(" ");
  Serial.print(ay, 2);
  Serial.print(" ");
  Serial.println(az, 2);

  Serial.print("Gyro: ");
  Serial.print(gx, 2);
  Serial.print(" ");
  Serial.print(gy, 2);
  Serial.print(" ");
  Serial.println(gz, 2);

  // ---------------- OLED ----------------

  display.clearDisplay();

  display.setCursor(0, 0);
  display.println("FLIGHT COMPUTER V1");

  display.setCursor(0, 14);
  display.print("ALT: ");
  display.print(relativeAltitude, 1);
  display.println(" m");

  display.setCursor(0, 26);
  display.print("TEMP:");
  display.print(temperature, 1);
  display.println(" C");

  display.setCursor(0, 38);
  display.print("A:");
  display.print(ax, 1);
  display.print(",");
  display.print(ay, 1);
  display.print(",");
  display.print(az, 1);

  display.setCursor(0, 50);
  display.print("G:");
  display.print(gx, 0);
  display.print(",");
  display.print(gy, 0);
  display.print(",");
  display.print(gz, 0);

  display.display();

  delay(200);
}
