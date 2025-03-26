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


// ultrasonic at front of robot
const int UF_ECHO_PIN = 29;
const int UF_TRIG_PIN = 28;
float UF_distance;

// ultrasonic radar
const int UR_ECHO_PIN = 30;
const int UR_TRIG_PIN = 31;
float UR_distance;




// --------- Setup --------- //

// runs once
void setup() {
  
  // set sensor pins
  pinMode(UF_TRIG_PIN, OUTPUT);
  pinMode(UF_ECHO_PIN, INPUT);

  Serial.begin(9600);

  // define servo pins
  radarServo.attach(RS_PIN);

}


// --------- Main Loop --------- //

// runs repeatedly
void loop() {

  
  // rotate servo
  for (int angle=RS_ANGLE_MIN; angle<=RS_ANGLE_MAX; angle+=RS_ANGLE_INTERVAL) {
    
    // get front and radar ultrasonic
    UF_distance = calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN);
    UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);


    // log front ultrasonic distance
    Serial.print("Distance: ");
    Serial.println(UF_distance);

    
    radarServo.write(angle);
    // Serial.println(angle);
    delay(20);

    checkQuit();
  }

  for (int angle=RS_ANGLE_MAX; angle>RS_ANGLE_MIN; angle-=RS_ANGLE_INTERVAL){
    
    // get front and radar ultrasonic
    UF_distance = calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN);
    UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);

    // log front ultrasonic distance
    Serial.print("Distance: ");
    Serial.println(UF_distance);
    
    
    radarServo.write(angle);
    // Serial.println(angle);
    delay(20);

    checkQuit();
  }

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







