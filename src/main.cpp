// SWITCHES

#define QTR_CALIBRARION 0
#define PRINT_SENSOR_VALUES 0

// FLAGS
bool whiteLineDetected = false;

bool start = false;

bool first_iterration = false;

volatile bool robotRunning = false;

// LIBRARIES
#include "Wire.h"
extern "C"
{
#include "utility/twi.h" // from Wire library, so we can do bus scanning
}
#include <SPI.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL6180X.h"
#include "QTRSensors.h"
#include "CytronMotorDriver.h"

// THE PIN ASSIGNMENTS

//  - IR module
#define IR_PIN 23

//  - Motor Driver
#define DIR1 9
#define PWM1 10
#define DIR2 11
#define PWM2 12
#define MAX_SPEED 255
#define NORMAL_SPEED 180
#define HALF_SPEED 128
const float WHEEL_DIAMETER = 3.0; // Diameter of the wheels in inches
const int RPM = 80;               // RPM of the motors

//   = I2C MUX
#define TCAADDR 0x70
#define TOFADDR 0x29

#define TOF_SENSOR_1 2 // channels for the sensors
#define TOF_SENSOR_2 3 // channels for the sensors
#define TOF_SENSOR_3 1 // channels for the sensors
#define TOF_SENSOR_4 4 // channels for the sensors
#define TOF_SENSOR_5 0 // channels for the sensors
#define TOF_SENSOR_6 5 // channels for the sensors
#define TOF_SENSOR_7 6 // channels for the sensors

//   - Analog MUX
const int SIG = 14;
const int S0 = 0;
const int S1 = 1;
const int S2 = 2;
const int S3 = 3;
const int NUM_CHANNEL = 15;
#define EmitterPin QTRNoEmitterPin;

#define backQTR_1 1
#define backQTR_2 2
#define backQTR_3 3
#define backQTR_4 4
#define backQTR_5 5
#define rightQTR_1 6
#define rightQTR_2 7
#define rightQTR_3 8
#define rightQTR_4 9
#define rightQTR_5 10
#define leftQTR_1 11
#define leftQTR_2 12
#define leftQTR_3 13
#define leftQTR_4 14
#define leftQTR_5 15

/////////////////ALL THE OBJECTS///////////////

// TOF sensor
Adafruit_VL53L0X lox[5]; // Aarray of 5 lox
// Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_VL6180X nox = Adafruit_VL6180X();

// Line sensor
const uint8_t SensorCount = 5; // Number of sensors per QTR
const uint8_t NumSensors = 3;  // Number of QTR sensors
QTRSensors qtrSensors[NumSensors];
uint16_t sensorValues[NumSensors][SensorCount]; // Array to store sensor values
uint16_t THRESHOLD = 400;

// Configure the motor driver.
CytronMD leftMotor(PWM_DIR, PWM1, DIR1);  // PWM1, DIR1
CytronMD rightMotor(PWM_DIR, PWM2, DIR2); // PWM2, DIR2

// 8888888888888888888888888888888888   MOTOR DRIVER FUNCTION   888888888888888888888888888888888888888888
void moveForward();
void moveBackward();
void slowmoveForward();
void slowmoveBackward();
void turnLeft();
void turnRight();
void stopRobot();
//         EXOTIC FUNCTIONS
void duration_slowMoveForward(unsigned long duration);
void turnDegrees(float degrees);

// 8888888888888888888888888888888888   LINE SENSORS FUNCTION   888888888888888888888888888888888888888888
void selectChannel(int channel);
void filling_the_array();
// void calibrateSensors();
bool isQTRDetected(int sensor);

bool isBackQTRDetected(uint16_t sensorValues[0][SensorCount]);
bool isRightQTRDetected(uint16_t sensorValues[1][SensorCount]);
bool isLeftQTRDetected(uint16_t sensorValues[2][SensorCount]);
void detectWhiteLine(uint16_t sensorValues[NumSensors][SensorCount]);

// 8888888888888888888888888888888888   TOF SENSORS FUNCTION   888888888888888888888888888888888888888888
void tcaselect(uint8_t i);

// 8888888888888888888888888888888888  FLAG and MAIN   888888888888888888888888888888888888888888
void reset_flags();
void read_all();
void react();

void handleInterrupt();

void setup()
{

  pinMode(SIG, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(IR_PIN, INPUT);
  // buildin led
  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(IR_PIN), handleInterrupt, CHANGE);

  Serial.begin(115200);
  Wire.begin();

  // wait until serial port opens for native USB devices

  // Initialize QTR sensors
  for (uint8_t i = 0; i < NumSensors; i++)
  {
    qtrSensors[i].setTypeAnalog();
    // Assigning pins for each QTR sensor
    uint8_t sensorPins[SensorCount] = {A0, A0, A0, A0, A0}; // Adjust pin assignments as per your setup
    qtrSensors[i].setSensorPins(sensorPins, SensorCount);
    qtrSensors[i].setEmitterPin(QTRNoEmitterPin); // Emitter control pin, adjust pin number as per your setup
  }
  // moveForward();

  //  for (int i = 2; i < 7; i++) {

  //  tcaselect(i);

  //   if (!lox[i].begin()) {
  //     Serial.print("Failed to boot VL53L0X SENSOR_ on channel");
  //     Serial.println(i);
  //     while (1);
  //   }
  //     Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));
  //     lox[i].startRangeContinuous();
  // }

  // power

  // start continuous ranging
}

int first = 0;

void loop()
{
  Serial.print(digitalRead(IR_PIN));
  int radar = digitalRead(IR_PIN);
  Serial.println(radar);
  if (!digitalRead(IR_PIN))
  {
    stopRobot();
    return;
  }
  digitalWrite(LED_BUILTIN, HIGH);

  filling_the_array();

  read_all();

  react();
}

// VL53L0X_RangingMeasurementData_t measure;

//   // Print the sensor values for this QTR sensor and emitter
//   // Print sensor values as a table
//   // printSensorValues(sensorValues);
//   // detectWhiteLine(sensorValues);

//   // tcaselect(TOF_SENSOR_1);
//   // lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

//   // if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//   //   Serial.print("L1: "); Serial.print(measure.RangeMilliMeter);Serial.print(" ");
//   // } else {
//   //   Serial.print("L1: "); Serial.print("OOR");Serial.print(" ");
//   // }

//   // tcaselect(TOF_SENSOR_2);
//   // lox.rangingTest(&measure, true); // pass in 'true' to get debug data printout!

//   // if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//   //   Serial.print("L2: "); Serial.print(measure.RangeMilliMeter);Serial.print(" ");
//   // } else {
//   //   Serial.print("L2: "); Serial.print("OOR");Serial.print(" ");
//   // }

//   // Serial.println(" ");

//   //  TOF sensor test
// for (int i = 2; i < 7; i++) {
//   tcaselect(i);

//   lox[i].rangingTest(&measure, false);
//   lox[i].readRange();
//   // if (measure.RangeStatus != 4) {          // phase failures have incorrect data
//   //   Serial.print("SENSOR_");
//   //   Serial.print(i );
//   //   Serial.print(": ");
//   //   Serial.print(lox[i].readRange());
//   //   Serial.print("mm    ");
//   // } else {
//   //   Serial.print("SENSOR_");
//   //   Serial.print(i );
//   //   Serial.println(": Out of Range");
//   // }
// delay(10);
// }
//     Serial.println(" ");
void handleInterrupt()
{
  if (digitalRead(IR_PIN) == 1)
  {
    Serial.print("Interupt");
    start = true;
    first_iterration = true;
  }
  else
  {
    start = false;
  }
}

void filling_the_array()
{
  // Loop through each channel of the multiplexer
  for (int channel = 1; channel < 16; channel++)
  {
    // Select the appropriate channel using the multiplexer
    selectChannel(channel);

    // Determine which sensor and emitter to read based on the current channel
    int sensorIndex = (channel - 1) / 5; // Calculate the sensor index (0, 1, or 2)

    // Read raw sensor values
    qtrSensors[sensorIndex].read(sensorValues[sensorIndex]);
  }
}

// //  TOF sensor test
//   for (int i = 0; i < 5; i++) {
//     tcaselect(TOF_SENSOR_1 + i);

//     lox[i].rangingTest(&measure, false);
//     if (measure.RangeStatus != 4) {          // phase failures have incorrect data
//       Serial.print("SENSOR_");
//       Serial.print(i + 1);
//       Serial.print(": ");
//       Serial.print(measure.RangeMilliMeter);
//       Serial.print("mm    ");
//     } else {
//       Serial.print("SENSOR_");
//       Serial.print(i + 1);
//       Serial.println(": Out of Range");
//     }
//   }
//       Serial.println(" ");
//   delay(500);

void reset_flags()
{

  whiteLineDetected = false;
  // all flags reset
}

bool isQTRDetected(int sensor)
{

  for (int j = 0; j < SensorCount; j++)
  {
    if (sensorValues[sensor][j] < THRESHOLD)
    {
      return true;
    }
    delay(5);
  }
  return false;
}

void read_all()
{
  if (isQTRDetected(0))
  {
    Serial.println("isBackQTRDetected");
  }
  if (isQTRDetected(1))
  {
    Serial.println("isleftQTRDetected");
  }
  if (isQTRDetected(2))
  {
    Serial.println("isrightQTRDetected");
  }
  Serial.println("Iteration");
}

// read all tof sensor

//   flag_tof_left = read_tof(tof1);
//  tofall r

// bool read_tof(tof_obg tof){
// //read tof
//   if(within range ){
//     return true //there is somethign
//   }
//   return false;
// }

void react()
{
  if ((isQTRDetected(1)) == true || (isQTRDetected(2)) == true)
  {
    moveBackward();
    delay(400);
    turnLeft();
    delay(400);
    moveForward();
  }
}
//     if(any of any tof){

//       if(tof1){
//         turnleft
//       }

//     }

// }

// 8888888888888888888888888888888888   MOTOR DRIVER FUNCTION   888888888888888888888888888888888888888888
void moveForward()
{
  leftMotor.setSpeed(MAX_SPEED);
  rightMotor.setSpeed(MAX_SPEED);
}

// Function to move the robot backward
void moveBackward()
{
  leftMotor.setSpeed(-MAX_SPEED);
  rightMotor.setSpeed(-MAX_SPEED);
}

// Function to move the robot forward
void slowmoveForward()
{
  leftMotor.setSpeed(NORMAL_SPEED);
  rightMotor.setSpeed(NORMAL_SPEED);
}

// Function to move the robot backward
void slowmoveBackward()
{
  leftMotor.setSpeed(-NORMAL_SPEED);
  rightMotor.setSpeed(-NORMAL_SPEED);
}

// Function to turn the robot left
void turnLeft()
{
  leftMotor.setSpeed(-MAX_SPEED);
  rightMotor.setSpeed(MAX_SPEED);
}

// Function to turn the robot right
void turnRight()
{
  leftMotor.setSpeed(MAX_SPEED);
  rightMotor.setSpeed(-MAX_SPEED);
}

// Function to stop the robot
void stopRobot()
{
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  robotRunning = false;
}

//         EXOTIC FUNCTIONS
// Function to move forward slowly for a specified duration
void duration_slowMoveForward(unsigned long duration)
{
  leftMotor.setSpeed(NORMAL_SPEED);
  rightMotor.setSpeed(NORMAL_SPEED);
  delay(duration);
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

// Function to make a specified degree turn
void turnDegrees(float degrees)
{
  float circumference = PI * WHEEL_DIAMETER;                               // Circumference of the wheel
  float distance = (circumference / 360.0) * degrees;                      // Distance to travel for the specified degree of rotation
  float timePerDegree = 60.0 / (RPM * 360.0 / 90.0);                       // Time per degree in minutes
  int duration = distance / circumference * 60.0 * timePerDegree * 1000.0; // Duration in milliseconds

  // Determine direction based on the sign of the degrees
  if (degrees > 0)
  {
    // Make the robot turn right
    turnRight();
  }
  else
  {
    // Make the robot turn left
    turnLeft();
  }

  // Wait for the turn to complete
  delay(abs(duration));

  // Stop the motors after the turn
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

// 8888888888888888888888888888888888   LINE SENSORS FUNCTION   888888888888888888888888888888888888888888
//  Function to select channel using multiplexer
void selectChannel(int channel)
{
  digitalWrite(S0, channel & 0x01);
  digitalWrite(S1, (channel >> 1) & 0x01);
  digitalWrite(S2, (channel >> 2) & 0x01);
  digitalWrite(S3, (channel >> 3) & 0x01);
}

bool isBackQTRDetected(uint16_t sensorValues[0][SensorCount])
{

  for (int j = 0; j < SensorCount; j++)
  {
    if (sensorValues[0][j] < THRESHOLD)
    {
      return true;
    }
  }
  return false;
}

bool isRightQTRDetected(uint16_t sensorValues[1][SensorCount])
{

  for (int j = 0; j < SensorCount; j++)
  {
    if (sensorValues[1][j] < THRESHOLD)
    {
      return true;
    }
  }
  return false;
}

bool isLeftQTRDetected(uint16_t sensorValues[2][SensorCount])
{
  for (int j = 0; j < SensorCount; j++)
  {
    if (sensorValues[2][j] < THRESHOLD)
    {
      return true;
    }
  }
  return false;
}
/*
// Function to calibrate sensors
#if QTR_CALIBRARION == 1
void calibrateSensors() {
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 1000; i++) {

    //calibrate three sensors
    for (uint8_t j = 0; j < NumSensors; j++) {
      qtrSensors[j].calibrate();
    }
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // Print the calibration minimum values measured when emitters were on

  for (uint8_t i = 0; i < NumSensors; i++) {
    for (uint8_t j = 0; j < SensorCount; j++) {
      Serial.print(qtrSensors[i].calibrationOn.minimum[j]);
      Serial.print(' ');
    }
    Serial.println();
  }

  // Print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < NumSensors; i++) {
    for (uint8_t j = 0; j < SensorCount; j++) {
      Serial.print(qtrSensors[i].calibrationOn.maximum[j]);
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
  delay(5000);
}
#else void calibrateSensors() {}
#endif

*/
#if PRINT_SENSOR_VALUES == 1
// Function Print the sensor values for this QTR sensor and emitter
void printSensorValues(uint16_t sensorValues[NumSensors][SensorCount])
{
  // Print column headers (emitter numbers)
  Serial.print("\tEmitter 1\tEmitter 2\tEmitter 3\tEmitter 4\tEmitter 5");
  Serial.println();

  // Print sensor values row by row
  for (int i = 0; i < NumSensors; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print("\t");

    for (int j = 0; j < SensorCount; j++)
    {
      Serial.print(sensorValues[i][j]);
      Serial.print("\t\t");
      // delay(500);
    }
    Serial.println();
  }
}
#else
void printSensorValues(uint16_t sensorValues[NumSensors][SensorCount])
{
}

#endif

void detectWhiteLine(uint16_t sensorValues[NumSensors][SensorCount])
{
  for (int i = 0; i < NumSensors; i++)
  {
    for (int j = 0; j < SensorCount; j++)
    {
      if (sensorValues[i][j] < THRESHOLD)
      {
        whiteLineDetected = true;
        Serial.print("White line detected at Sensor ");
        Serial.print(i + 1); // Sensor numbering starting from 1
        Serial.print(", Emitter ");
        Serial.print(j + 1); // Emitter numbering starting from 1
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }
}

// 8888888888888888888888888888888888   TOF SENSORS FUNCTION   888888888888888888888888888888888888888888

void tcaselect(uint8_t i)
{
  if (i < 0 && i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
