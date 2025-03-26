#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// --------- Function Prototypes --------- //
void checkQuit();
float calculateDistance(const int, const int);
void calculateData(int);



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



// --------- Setup --------- //

// runs once
//Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  
  // set sensor pins
  pinMode(UR_TRIG_PIN, OUTPUT);
  pinMode(UR_ECHO_PIN, INPUT);
  pinMode(UF_TRIG_PIN, OUTPUT);
  pinMode(UF_ECHO_PIN, INPUT);

  Serial.begin(100000);

  // define servo pins
  radarServo.attach(RS_PIN);

//   if (!mlx.begin()) {
//     Serial.println("Error connecting to MLX90614! Check wiring.");
//     while(1);

// }
}



// --------- Main Loop --------- //
// runs repeatedly
void loop() {

  
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

}




// --------- Functions --------- //

void calculateData(int angle){

  String data = ("A:" + String(angle) + "DS:" + String(calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN)) + "DA:" + String(calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN)));
  //"AT:", String(mlx.readAmbientTempC()) + "OT:" + String(mlx.readObjectTempC())
  Serial.println(data);
}


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



