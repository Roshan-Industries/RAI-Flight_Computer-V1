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

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found!");
    while (1);
  }

  // BMP280
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 not found!");
      while (1);
    }
  }

  // Wake IMU
  imuWrite(0x6B, 0x00);
  delay(100);

  Serial.println("Integrated sensor test started.");
}

void loop() {

  // -------- BMP280 --------
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(1013.25);

  // -------- IMU --------
  int16_t axRaw = imuRead16(0x3B);
  int16_t ayRaw = imuRead16(0x3D);
  int16_t azRaw = imuRead16(0x3F);

  int16_t gxRaw = imuRead16(0x43);
  int16_t gyRaw = imuRead16(0x45);
  int16_t gzRaw = imuRead16(0x47);

  // Assuming MPU6050-compatible default scales:
  // Accelerometer ±2g -> 16384 LSB/g
  // Gyroscope ±250 deg/s -> 131 LSB/(deg/s)

  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float gx = gxRaw / 131.0;
  float gy = gyRaw / 131.0;
  float gz = gzRaw / 131.0;

  // -------- Serial Monitor --------

  Serial.println("--------- FLIGHT COMPUTER TEST ---------");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Accel X/Y/Z: ");
  Serial.print(ax);
  Serial.print(" / ");
  Serial.print(ay);
  Serial.print(" / ");
  Serial.println(az);

  Serial.print("Gyro X/Y/Z: ");
  Serial.print(gx);
  Serial.print(" / ");
  Serial.print(gy);
  Serial.print(" / ");
  Serial.println(gz);

  // -------- OLED --------

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("Flight Computer V1");

  display.setCursor(0, 14);
  display.print("Alt: ");
  display.print(altitude, 1);
  display.println(" m");

  display.setCursor(0, 26);
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println(" C");

  display.setCursor(0, 38);
  display.print("AX:");
  display.print(ax, 2);

  display.print(" AY:");
  display.print(ay, 2);

  display.setCursor(0, 50);
  display.print("AZ:");
  display.print(az, 2);

  display.display();

  delay(500);
}
