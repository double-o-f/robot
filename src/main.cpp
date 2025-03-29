#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



// --------- Function Prototypes --------- //
void checkQuit();
float calculateDistance(const int, const int);
void sendData(int);
void rotateMotor(const int, const int, const int);
void stopMotors();

float getYawAngle();
void calibrateGyro();
void setAngle(int);
// --------- Variables --------- //

// radar servo (RS)
Servo radarServo;
const int RS_PIN = 10;
const int RS_ANGLE_MIN = 0;
const int RS_ANGLE_MAX = 180;
int RS_ANGLE_INTERVAL = 2;
int radarAngle = 90;  // used to keep track of radar servo pos


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


// MPU6050
Adafruit_MPU6050 mpu;
float yawAngle = 0;
unsigned long previousTime = 0;
float gyroZOffset = 0;

// ir sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();




// --------- Setup --------- //

// runs once


void setup() {
  
  Serial.begin(115200);

  
  // set sensor pins
  pinMode(UR_TRIG_PIN, OUTPUT);
  pinMode(UR_ECHO_PIN, INPUT);
  pinMode(UF_TRIG_PIN, OUTPUT);
  pinMode(UF_ECHO_PIN, INPUT);

  // define servo pins
  radarServo.attach(RS_PIN);
  radarServo.write(radarAngle); // set radar to 90 to start (straight ahead)


  // Define wheel motor pins
  pinMode(WHEEL_ENA, OUTPUT);
  pinMode(WHEEL_IN1, OUTPUT);
  pinMode(WHEEL_IN2, OUTPUT);

  mpu.begin(); //this line likes to cause hang
  calibrateGyro();

  // can try different range for more accurate readings (2-16)
  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // not need if no use acceleromter

}



// maybe
// global target angle, gets updated by pi slam algorithm, robot moves as data is still being communicated
int target_angle = 20;  // 20 for testing
int robot_angle = 0;   // i think implementation is on connor computer

// different implementation of setAngle()
void turnToAngle(int target_angle) {

  robot_angle = getYawAngle();

  if (robot_angle < target_angle) {
    // turn right with left wheel faster
    digitalWrite(WHEEL_IN1, HIGH);  // Motor A forward
    digitalWrite(WHEEL_IN2, LOW);   // Motor A not reverse
    analogWrite(WHEEL_ENA, 255);

    digitalWrite(WHEEL_IN3, LOW);   // Motor B not forward
    digitalWrite(WHEEL_IN4, HIGH);  // Motor B reverse
    analogWrite(WHEEL_ENB, 20);

  } 
  else if (robot_angle > target_angle) {
    // turn left with right wheel faster
    digitalWrite(WHEEL_IN1, LOW);   // Motor A not forward
    digitalWrite(WHEEL_IN2, HIGH);  // Motor A reverse
    analogWrite(WHEEL_ENA, 20);

    digitalWrite(WHEEL_IN3, HIGH);  // Motor B forward
    digitalWrite(WHEEL_IN4, LOW);   
    analogWrite(WHEEL_ENB, 255);
  }
  else {

  }
}



void loop() {

  turnToAngle(target_angle);







}



void loop() {
  //sendData(); //check all sensors and send data to pi

  // update radar angle
  // if (radarAngle >= RS_ANGLE_MAX || radarAngle <= RS_PIN)
  //   RS_ANGLE_INTERVAL = -RS_ANGLE_INTERVAL;

  // radarAngle += RS_ANGLE_INTERVAL;
  setAngle(20);
  delay(1000);
  setAngle(190);
  delay(1000);
}



void calibrateGyro() {
  const int numSamples = 100;
  for (int i = 0; i < numSamples; i++) {
      sensors_event_t accelEvent, gyroEvent, tempEvent;
      mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
      gyroZOffset += gyroEvent.gyro.z;
      delay(10); // Wait for sensor to stabilize
  }
  gyroZOffset /= numSamples;
}

void setAngle(int angle){
  int yaw = getYawAngle();
  while(getYawAngle() != angle){
    
    if (angle < 180){
      digitalWrite(WHEEL_IN1, HIGH);  // Motor A forward
      digitalWrite(WHEEL_IN2, LOW);   // Motor A not reverse

      digitalWrite(WHEEL_IN3, LOW);   // Motor B not forward
      digitalWrite(WHEEL_IN4, HIGH);  // Motor B reverse
    }

    if(angle >= 180){
      digitalWrite(WHEEL_IN1, LOW);   // Motor A not forward
      digitalWrite(WHEEL_IN2, HIGH);  // Motor A reverse
  
      digitalWrite(WHEEL_IN3, HIGH);  // Motor B forward
      digitalWrite(WHEEL_IN4, LOW);   
    }
    analogWrite(WHEEL_ENA, 255);
    analogWrite(WHEEL_ENB, 255);
  }
}


// // --------- Main Loop --------- //
// // runs repeatedly
// void loop() {

  

//   rotateMotor(WHEEL_ENA, WHEEL_IN1, WHEEL_IN2);
//   rotateMotor(WHEEL_ENB, WHEEL_IN3, WHEEL_IN4);
//   checkQuit();

  
//   // rotate servo
//   for (int angle=RS_ANGLE_MIN; angle<=RS_ANGLE_MAX; angle+=RS_ANGLE_INTERVAL) {
//     calculateData(angle);
   
//     radarServo.write(angle);

//     delay(20); 
//     // check and save distance
//     // UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);

//     checkQuit();
//   }

//   for (int angle=RS_ANGLE_MAX; angle>RS_ANGLE_MIN; angle-=RS_ANGLE_INTERVAL){
//     calculateData(angle);
//     radarServo.write(angle);
//     delay(20);

//     checkQuit();
//   }

//   while (true)
//   {
//     stopMotors();
//   }
  

// }




// --------- Functions --------- //

// return mpu6050 angle
float getYawAngle() {

  unsigned long currentTime = millis(); // Get current time in milliseconds
  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  sensors_event_t accel, gyro, temp;

  mpu.getEvent(&accel, &gyro, &temp); // Retrieve sensor events

  float corrected_gyro = gyro.gyro.z - gyroZOffset;

  yawAngle += corrected_gyro * deltaTime * (180/PI);
  yawAngle = fmod(yawAngle, 360); // Keep within 360 degrees
  if (yawAngle < 0) yawAngle += 360;

  return yawAngle;
}

void stopMotors() {
  digitalWrite(WHEEL_IN1, LOW);
  digitalWrite(WHEEL_IN2, LOW);
  analogWrite(WHEEL_ENA, 0);

  digitalWrite(WHEEL_IN3, LOW);
  digitalWrite(WHEEL_IN4, LOW);
  analogWrite(WHEEL_ENB, 0);
}

void rotateMotor(const int EN, const int IN1, const int IN2) {

  /*

  ENA pin -- sets speed
  IN1 / IN2 -- sets direction
  
  IN1 - H and IN2 - L == forward
  IN1 - L and IN2 - H == backward
  
  */

  // Rotate motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN, 255); // Set speed (0-255)

}


void setMotorSpeeds(int angle) {

  // move left wheel faster and right slower for turning right


  // move right wheel faster and left slower for turning left



  // for above implementation
  // analogWrite(WHEEL_ENA, 255); // Set speed (0-255)
  // analogWrite(WHEEL_ENB, 255); // Set speed (0-255)

}


void move(String direction, int angle) {

  if (direction == "foward") {

    // makes motors go in forward direction
    digitalWrite(WHEEL_IN1, HIGH); // IN1 is 
    digitalWrite(WHEEL_IN2, LOW);

    digitalWrite(WHEEL_IN3, LOW);
    digitalWrite(WHEEL_IN4, HIGH);

    // take the angle and determine how much the right vs left motors should turn
    setMotorSpeeds(angle);


  } 

  else if (direction == "reverse") {

    // makes motors go in reverse direction
    digitalWrite(WHEEL_IN1, LOW);
    digitalWrite(WHEEL_IN2, HIGH);

    digitalWrite(WHEEL_IN3, HIGH);
    digitalWrite(WHEEL_IN4, LOW);

    // take the angle and determine how much the right vs left motors should turn
    setMotorSpeeds(angle);


  }


}







void mvForward() {
  digitalWrite(WHEEL_IN1, HIGH);
  digitalWrite(WHEEL_IN2, LOW);
  analogWrite(WHEEL_ENA, 255); // Set speed (0-255)

  digitalWrite(WHEEL_IN3, LOW);
  digitalWrite(WHEEL_IN4, HIGH);
  analogWrite(WHEEL_ENB, 255); // Set speed (0-255)
}

void mvBackward() {
  digitalWrite(WHEEL_IN1, LOW);
  digitalWrite(WHEEL_IN2, HIGH);
  analogWrite(WHEEL_ENA, 255); // Set speed (0-255)

  digitalWrite(WHEEL_IN3, HIGH);
  digitalWrite(WHEEL_IN4, LOW);
  analogWrite(WHEEL_ENB, 255); // Set speed (0-255)
}

void trnLeft() {
  digitalWrite(WHEEL_IN1, LOW);
  digitalWrite(WHEEL_IN2, HIGH);
  analogWrite(WHEEL_ENA, 255); // Set speed (0-255)

  digitalWrite(WHEEL_IN3, LOW);
  digitalWrite(WHEEL_IN4, HIGH);
  analogWrite(WHEEL_ENB, 255); // Set speed (0-255)
}

void trnRight() {
  digitalWrite(WHEEL_IN1, HIGH);
  digitalWrite(WHEEL_IN2, LOW);
  analogWrite(WHEEL_ENA, 255); // Set speed (0-255)

  digitalWrite(WHEEL_IN3, HIGH);
  digitalWrite(WHEEL_IN4, LOW);
  analogWrite(WHEEL_ENB, 255); // Set speed (0-255)
}


void sendData(){

  String data = ("AG:" + String(radarAngle)                                  + ":AG-" + 
                 "DF:" + String(calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN)) + ":DF-" + 
                 "DP:" + String(calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN)) + ":DP-" + 
                 //"TR:" + String(mlx.readAmbientTempC())                      + ":TR-" +
                 //"TO:" + String(mlx.readObjectTempC())                       + ":TO-" +
                 "YA:" + String(int(getYawAngle() - gyroZOffset))            + ":YA-");

  Serial.println(data);

//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);
//   Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
//   Serial.print(" | Accel Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2");
//   Serial.print(" | Accel Z: "); Serial.print(a.acceleration.z); Serial.print(" m/s^2");
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



