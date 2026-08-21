// ============================================================
// FC V1 - LED ONLY TEST
// ============================================================

#define LED_GREEN   26
#define LED_YELLOW  27
#define LED_BLUE    32
#define LED_RED     33

void setup()
{
  Serial.begin(115200);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // Start with everything OFF
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.println("LED TEST START");
}

void loop()
{
  // GREEN
  Serial.println("GREEN ON");

  digitalWrite(LED_GREEN, HIGH);
  delay(1000);

  digitalWrite(LED_GREEN, LOW);
  delay(500);


  // YELLOW
  Serial.println("YELLOW ON");

  digitalWrite(LED_YELLOW, HIGH);
  delay(1000);

  digitalWrite(LED_YELLOW, LOW);
  delay(500);


  // BLUE
  Serial.println("BLUE ON");

  digitalWrite(LED_BLUE, HIGH);
  delay(1000);

  digitalWrite(LED_BLUE, LOW);
  delay(500);


  // RED
  Serial.println("RED ON");

  digitalWrite(LED_RED, HIGH);
  delay(1000);

  digitalWrite(LED_RED, LOW);
  delay(500);


  // ALL ON
  Serial.println("ALL LEDs ON");

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  delay(2000);


  // ALL OFF
  Serial.println("ALL LEDs OFF");

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);

  delay(2000);
}
