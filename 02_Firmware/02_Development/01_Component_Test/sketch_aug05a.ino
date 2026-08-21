#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("RAI OLED DISPLAY TEST");

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED initialization FAILED");
    Serial.println("Check wiring and I2C address.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("OLED initialization SUCCESSFUL");

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(18, 0);
  display.println("ROSHAN");

  display.setTextSize(1);
  display.setCursor(31, 24);
  display.println("AEROSPACE");

  display.setCursor(28, 36);
  display.println("INDUSTRIES");

  display.setCursor(20, 52);
  display.println("UAV_V1");

  display.display();

  Serial.println("Text displayed successfully.");
}

void loop() {
  // Nothing required for this basic test.
}
