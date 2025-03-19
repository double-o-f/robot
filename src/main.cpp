#include <Arduino.h>
// #inlcude <Servo.h>


// --------- Function Prototypes --------- //
int calculateDistance(int, int);


// --------- Variables --------- //

// ultrasonic at front of robot
const int UF_ECHO_PIN = 23;
const int UF_TRIG_PIN = 25;
int UF_distance;

// ultrsonic radar
// const int UR_ECHO_PIN = ;
// const int UR_TRIG_PIN = ;


// --------- Setup --------- //

// runs once
void setup() {
  
  // set sensor pins
  pinMode(UF_TRIG_PIN, OUTPUT);
  pinMode(UF_ECHO_PIN, INPUT);

  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
}


// --------- Main Loop --------- //

// runs repeatedly
void loop() {

  UF_distance = calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN);
  Serial.print("Distance: ");
  Serial.println(UF_distance);
  delay(10);

}


// --------- Functions --------- //

// calculates distance for a given sensor
int calculateDistance(int trig_pin, int echo_pin) {

  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // reads echo_pin, returns sound wave travel time in ms
  long duration = pulseIn(echo_pin, HIGH);
  int distance = duration*0.034/2;
  return distance;

}









