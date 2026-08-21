#include <Wire.h>
#include <Adafruit_BMP280.h>

// Create the BMP280 sensor object
Adafruit_BMP280 bmp;

// Common BMP280 I2C addresses
const uint8_t BMP_ADDRESS_1 = 0x76;
const uint8_t BMP_ADDRESS_2 = 0x77;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("RAI BMP280 SENSOR TEST");
  Serial.println("----------------------");

  // ESP32 default I2C pins:
  // SDA = GPIO 21
  // SCL = GPIO 22
  Wire.begin(21, 22);

  bool sensorFound = false;

  // First try address 0x76
  if (bmp.begin(BMP_ADDRESS_1)) {
    Serial.println("BMP280 detected at I2C address 0x76");
    sensorFound = true;
  }
  // If that fails, try address 0x77
  else if (bmp.begin(BMP_ADDRESS_2)) {
    Serial.println("BMP280 detected at I2C address 0x77");
    sensorFound = true;
  }

  if (!sensorFound) {
    Serial.println("BMP280 NOT FOUND");
    Serial.println("Check:");
    Serial.println("1. VCC -> 3.3V");
    Serial.println("2. GND -> GND");
    Serial.println("3. SCL -> GPIO 22");
    Serial.println("4. SDA -> GPIO 21");
    Serial.println("5. Header-pin contact");
    Serial.println("6. I2C address");
    
    while (true) {
      delay(1000);
    }
  }

  // Basic sensor configuration
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );

  Serial.println("BMP280 initialization successful.");
  Serial.println();
}

void loop() {
  float temperature = bmp.readTemperature();
  float pressure_hPa = bmp.readPressure() / 100.0F;

  // 1013.25 hPa is standard sea-level pressure.
  // This altitude is only approximate unless local sea-level pressure is calibrated.
  float altitude = bmp.readAltitude(1013.25F);

  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure_hPa, 2);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude: ");
  Serial.print(altitude, 2);
  Serial.println(" m");

  Serial.println("----------------------");

  delay(2000);
}
