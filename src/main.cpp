#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// --------- Function Prototypes --------- //
void checkQuit();
int calculateDistance(int, int);



// --------- Variables --------- //

// radar servo (RS)
Servo radarServo;
const int RS_PIN = 10;
const int RS_ANGLE_MIN = 0;
const int RS_ANGLE_MAX = 180;
const int RS_ANGLE_INTERVAL = 2;


// ultrasonic radar
const int UR_ECHO_PIN = 23;
const int UR_TRIG_PIN = 25;
float UR_distance;

// ultrasonic at front of robot
// const int UF_ECHO_PIN = ;
// const int UF_TRIG_PIN = ;




// --------- Setup --------- //

// runs once
void setup() {
  
  // set sensor pins
  pinMode(UR_TRIG_PIN, OUTPUT);
  pinMode(UR_ECHO_PIN, INPUT);

  Serial.begin(9600);

  // define servo pins
  radarServo.attach(RS_PIN);

}


// --------- Main Loop --------- //

// runs repeatedly
void loop() {

  
  // rotate servo
  for (int angle=RS_ANGLE_MIN; angle<=RS_ANGLE_MAX; angle+=RS_ANGLE_INTERVAL) {
    radarServo.write(angle);
    Serial.println(angle);
    delay(20);

    // check and save distance
    // UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);

    checkQuit();
  }

  for (int angle=RS_ANGLE_MAX; angle>RS_ANGLE_MIN; angle-=RS_ANGLE_INTERVAL){
    radarServo.write(angle);
    Serial.println(angle);
    delay(20);

    // check and save distance
    // UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);

    checkQuit();
  }

  // USE THIS FOR FRONT ONE
  // UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);
  // Serial.print("Distance: ");
  // Serial.println(UR_distance);
  // delay(10);

}




// --------- Functions --------- //


// quit running if q is pressed
void checkQuit() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'q') { // 'Q' stands for "quit"
      Serial.print("Program Stopped");
      while (true); // Infinite loop stops the program
    }
  }
}

// calculates distance for a given sensor
int calculateDistance(int trig_pin, int echo_pin) {

  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // reads echo_pin, returns sound wave travel time in ms
  long duration = pulseIn(echo_pin, HIGH);
  return duration*0.0343/2; // convert to cm

}







