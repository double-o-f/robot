#include <Arduino.h>
#include <Servo.h>

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


// --------- Variables --------- //

// radar servo (RS)
Servo radar_servo;
const int RS_PIN = 27;
const int RS_ANGLE_MIN = 15;
const int RS_ANGLE_MAX = 165;

// ultrasonic at front of robot
const int UF_ECHO_PIN = 23;
const int UF_TRIG_PIN = 25;
float UF_distance;
float UF_data[RS_ANGLE_MAX - RS_ANGLE_MIN + 1]; // 151 for 15 to 165 degrees


// ultrsonic radar
// const int UR_ECHO_PIN = ;
// const int UR_TRIG_PIN = ;




// --------- Setup --------- //

// runs once
void setup() {
  
  // set sensor pins
  pinMode(UF_TRIG_PIN, OUTPUT);
  pinMode(UF_ECHO_PIN, INPUT);

  // initialize data arrays
  initializeData(UF_data);

  Serial.begin(9600);

  // define radar servo pin
  radar_servo.attach(RS_PIN);
}


// --------- Main Loop --------- //

// runs repeatedly
void loop() {

  // rotate servo
  for (int i=15; i<=165; i++) {
    radar_servo.write(i);
    delay(15);

    // check and save distance
    UF_distance = calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN);
    storeRadarData(i, UF_distance);
    Serial.print(i);
    Serial.print(" -- Distance: ");
    Serial.println(UF_distance);
  }

  for (int i=165; i>15; i--){
    radar_servo.write(i);
    delay(15);

    // check and save distance
    UF_distance = calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN);
    storeRadarData(i, UF_distance);
    Serial.print(i);
    Serial.print(" -- Distance: ");
    Serial.println(UF_distance);
  }

  // UF_distance = calculateDistance(UF_TRIG_PIN, UF_ECHO_PIN);
  // Serial.print("Distance: ");
  // Serial.println(UF_distance);
  // delay(10);

  checkQuit();

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
  int distance = duration*0.034/2;
  return distance;

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
    UF_data[index] = distance;
  }

}

// will be used for visualization
float getRadarData(int angle) {

  if (angle >= RS_ANGLE_MIN && angle < RS_ANGLE_MAX) {
    int index = angle - RS_ANGLE_MIN;
    return UF_data[index];
  }
  return -1;
}










