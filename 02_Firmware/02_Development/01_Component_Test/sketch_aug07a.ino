#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found at 0x76");
    Serial.println("Trying 0x77...");

    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 not found!");
      while (1);
    }
  }

  Serial.println("BMP280 connected successfully!");
}

void loop() {
  Serial.print("Temperature: ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(bmp.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude: ");
  Serial.print(bmp.readAltitude(1013.25));
  Serial.println(" m");

  Serial.println("----------------------");

  delay(2000);
}
