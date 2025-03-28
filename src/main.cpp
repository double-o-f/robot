#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// --------- Function Prototypes --------- //
void checkQuit();
float calculateDistance(const int, const int);
void calculateData(int);
void rotateMotor(const int, const int, const int);
void stopMotors();


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
const int UR_ECHO_PIN = 31;
const int UR_TRIG_PIN = 30;
float UR_distance;


// Wheel Motors
const int WHEEL_ENA = 6;
const int WHEEL_IN1 = 22;
const int WHEEL_IN2 = 23;

const int WHEEL_ENB = 7;
const int WHEEL_IN3 = 24;
const int WHEEL_IN4 = 25;



// --------- Setup --------- //

// runs once
//Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  
  // set sensor pins
  pinMode(UR_TRIG_PIN, OUTPUT);
  pinMode(UR_ECHO_PIN, INPUT);
  pinMode(UF_TRIG_PIN, OUTPUT);
  pinMode(UF_ECHO_PIN, INPUT);

  // define servo pins
  radarServo.attach(RS_PIN);

  // Define wheel motor pins
  pinMode(WHEEL_ENA, OUTPUT);
  pinMode(WHEEL_IN1, OUTPUT);
  pinMode(WHEEL_IN2, OUTPUT);



//   if (!mlx.begin()) {
//     Serial.println("Error connecting to MLX90614! Check wiring.");
//     while(1);

// }

  Serial.begin(9600);
}



// --------- Main Loop --------- //
// runs repeatedly
void loop() {

  rotateMotor(WHEEL_ENA, WHEEL_IN1, WHEEL_IN2);
  rotateMotor(WHEEL_ENB, WHEEL_IN3, WHEEL_IN4);
  checkQuit();

  
  // rotate servo
  for (int angle=RS_ANGLE_MIN; angle<=RS_ANGLE_MAX; angle+=RS_ANGLE_INTERVAL) {
    calculateData(angle);
   
    radarServo.write(angle);

    delay(20); 
    // check and save distance
    // UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);

    checkQuit();
  }

  for (int angle=RS_ANGLE_MAX; angle>RS_ANGLE_MIN; angle-=RS_ANGLE_INTERVAL){
    calculateData(angle);
    radarServo.write(angle);
    delay(20);

    checkQuit();
  }

  while (true)
  {
    stopMotors();
  }
  

}




// --------- Functions --------- //

void stopMotors() {
  digitalWrite(WHEEL_IN1, LOW);
  digitalWrite(WHEEL_IN2, LOW);
  analogWrite(WHEEL_ENA, 0);

  digitalWrite(WHEEL_IN3, LOW);
  digitalWrite(WHEEL_IN4, LOW);
  analogWrite(WHEEL_ENB, 0);
}

void rotateMotor(const int ENA, const int IN1, const int IN2) {

  /*

  ENA pin -- sets speed
  IN1 / IN2 -- sets direction
  
  IN1 - H and IN2 - L == forward
  IN1 - L and IN2 - H == backward
  
  */

  // Rotate motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255); // Set speed (0-255)

  // get rid of delay and it should run while other code executes
  // delay(2000); // Run for 2 seconds

  // delay(20); // Wait for 2 seconds

  // Rotate motor backward
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // analogWrite(ENA, 255); // Set speed (0-255)

  // delay(2000); // Run for 2 seconds

  // // Stop motor
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, LOW);
  // analogWrite(ENA, 0); // Stop motor

  // delay(2000); // Wait for 2 seconds


}

void calculateData(int angle){
  String data = ("AG:" + String(angle)                                       + ":AG-" + 
                 "DF:" + String(calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN)) + ":DF-" + 
                 "DP:" + String(calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN)) + ":DP-" + 
                 "TR:" + String(mlx.readAmbientTempC())                      + ":TR-" +
                 "TO:" + String(mlx.readObjectTempC())                       + ":TO");
  Serial.println(data);
}


// quit running if q is pressed
void checkQuit() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'q') { // 'Q' stands for "quit"
      Serial.print("Program Stopped");
      stopMotors();
      while (true); // Infinite loop stops the program
    }
  }
}

// calculates distance for a given sensor
float calculateDistance(const int trig_pin, const int echo_pin) {

  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // reads echo_pin, returns sound wave travel time in ms
  float duration = pulseIn(echo_pin, HIGH);
  return duration*0.0343/2; // convert to cm

}



