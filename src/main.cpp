#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// LCD pins in User_Setup.h
// #define TFT_MISO 50
// #define TFT_MOSI 51
// #define TFT_SCLK 52
// #define TFT_CS   10  // Chip select control pin
// #define TFT_DC    9  // Data Command control pin
// #define TFT_RST   8  // Reset pin (could connect to RST pin)
//#define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST

// may need to reduce spi_frequency in User_Setup.h for stable connection
// #define SPI_FREQUENCY 2000000 // 2 MHz (adjust as needed)



// --------- Function Prototypes --------- //
void checkQuit();
int calculateDistance(int, int);

void initializeData(float[]);
void storeRadarData(int, float);
float getRadarData(int);
void displayRadarData(int, float);


// --------- Variables --------- //

// ir sensor
Adafruit_MLX90614 IR_Sensor = Adafruit_MLX90614();

// radar servo (RS)
Servo radarServo;
const int RS_PIN = 27;
const int RS_ANGLE_MIN = 15;
const int RS_ANGLE_MAX = 165;

// spool servo (SS)
Servo spoolServo;
const int SS_PIN = 9;

// ultrasonic radar
const int UR_ECHO_PIN = 23;
const int UR_TRIG_PIN = 25;
float UR_distance;
float UR_data[RS_ANGLE_MAX - RS_ANGLE_MIN + 1]; // 151 for 15 to 165 degrees


// ultrasonic at front of robot
// const int UF_ECHO_PIN = ;
// const int UF_TRIG_PIN = ;




// --------- Setup --------- //

// runs once
void setup() {
  
  // set sensor pins
  pinMode(UR_TRIG_PIN, OUTPUT);
  pinMode(UR_ECHO_PIN, INPUT);

  // initialize data arrays
  initializeData(UR_data);

  Serial.begin(9600);

  // define servo pins
  radarServo.attach(RS_PIN);
  spoolServo.attach(SS_PIN);

  if (!IR_Sensor.begin()) {
    Serial.println("Failed to initialize MLX90614 sensor! Check wiring.");
    while (1); // Halt if sensor initialization fails
}


}


// --------- Main Loop --------- //

// runs repeatedly
void loop() {


  // read ambient temp (surroundings)
  float ambientTemp = IR_Sensor.readAmbientTempC();

  // read object temp (target)
  float objectTemp = IR_Sensor.readObjectTempC();

  // Print to Serial Monitor
  Serial.print("Ambient Temperature: ");
  Serial.print(ambientTemp);
  Serial.println(" °C");

  Serial.print("Object Temperature: ");
  Serial.print(objectTemp);
  Serial.println(" °C");

  delay(1000); // Wait 1 second before the next reading

  
  // // rotate servo
  // for (int angle=RS_ANGLE_MIN; angle<=RS_ANGLE_MAX; angle++) {
  //   radarServo.write(angle);
  //   spoolServo.write(angle);
  //   delay(20);

  //   // check and save distance
  //   UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);
  //   storeRadarData(angle, UR_distance); // store in array

  //   checkQuit();
  // }

  // for (int angle=RS_ANGLE_MAX; angle>RS_ANGLE_MIN; angle--){
  //   radarServo.write(angle);
  //   spoolServo.write(angle);
  //   delay(20);

  //   // check and save distance
  //   UR_distance = calculateDistance(UR_TRIG_PIN, UR_ECHO_PIN);
  //   storeRadarData(angle, UR_distance); // store in array

  //   checkQuit();
  // }

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



// ----- Radar Data Functions ----- //


// initialize all data to -1
void initializeData(float array[]) {
  for (int i = 0; i < (RS_ANGLE_MAX - RS_ANGLE_MIN + 1); i++) {
    array[i] = -1.0; // Default value
  }
}

// store data into array
void storeRadarData(int angle, float distance) {

  if (angle >= RS_ANGLE_MIN && angle <= RS_ANGLE_MAX) {
    int index = angle - RS_ANGLE_MIN;
    UR_data[index] = distance;
  }

}

// will be used for visualization
float getRadarData(int angle) {

  if (angle >= RS_ANGLE_MIN && angle < RS_ANGLE_MAX) {
    int index = angle - RS_ANGLE_MIN;
    return UR_data[index];
  }
  return -1;
}









