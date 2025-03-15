#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
uint8_t ledToggle(uint8_t);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  Serial.begin(9600);
  Serial.print(result);
  Serial.println(" balls");

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(ledToggle(LED_BUILTIN));
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

uint8_t ledToggle(uint8_t pin) {
  uint8_t state = !digitalRead(pin);
  digitalWrite(pin, state);
  return state;
}
