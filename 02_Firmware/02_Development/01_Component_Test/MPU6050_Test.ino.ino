#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("RAI MPU6050 SENSOR TEST");
  Serial.println("-----------------------");

  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND");
    Serial.println("Check VCC, GND, SDA, SCL and header contact.");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 detected successfully.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Sensor configuration complete.");
  Serial.println();
}

void loop() {
  sensors_event_t acceleration;
  sensors_event_t rotation;
  sensors_event_t temperature;

  mpu.getEvent(&acceleration, &rotation, &temperature);

  Serial.println("ACCELERATION (m/s^2)");

  Serial.print("X: ");
  Serial.print(acceleration.acceleration.x, 2);

  Serial.print(" | Y: ");
  Serial.print(acceleration.acceleration.y, 2);

  Serial.print(" | Z: ");
  Serial.println(acceleration.acceleration.z, 2);

  Serial.println("GYROSCOPE (rad/s)");

  Serial.print("X: ");
  Serial.print(rotation.gyro.x, 2);

  Serial.print(" | Y: ");
  Serial.print(rotation.gyro.y, 2);

  Serial.print(" | Z: ");
  Serial.println(rotation.gyro.z, 2);

  Serial.print("Temperature: ");
  Serial.print(temperature.temperature, 2);
  Serial.println(" °C");

  Serial.println("-----------------------");

  delay(1000);
}
