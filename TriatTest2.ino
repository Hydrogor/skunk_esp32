// 12-21-23 added different modes
// 1-2-24 012-003 added ESP32 availability
// 1-3-24 012-004 added ESP32 SD card access
// 1-4-24 012-005 added ESP32 SD read capability
// 1-5-24 012-006 added timer for ESP32 for recording in fixed samples
// 1-8-24 012-007 hooking up to robot, getting DX1000 location chip to talk
// 1-9-24 012-008 looking at why the I2C ultrasonic sensors are slowing everying down.
// 1-10-24 012-009 1 sensor was bad, am redoing without mux, fixed interupt issue
// 1-10-24 012-011 installing sensors
// 1-17-24 012-011 changed motor 2 direction due to re wiring, also steering
// 1-26-24 013-001 added. IMU, incorporated heading error (bno z - ThetaPath)*constantHeading to LH and RH outputs during playback
// 2-05-24 013-002 added fitness equation
// 3-18-24 014-001 changed motor controllers to solo uno, have to use analog out and incorporate a direction pin
// 3-20-24 014-002 blocked out servo code
// 5-22-24 014-004 added mood lights and cleaned up code
// 6-07-24 014-005 implementing error into playback drive
// 6-13-24 015-001 updated commands from updated ESP32
// 7-12-24 016-001 went back to rc pwm motor controllers
#define ESP32
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MovingAverage.h"
#include "SparkFun_Qwiic_Ultrasonic_Arduino_Library.h"
#include "SparkFun_TCA9534.h"
#include <ESP32Servo.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <EasyBNO055_ESP.h>  //bno IMU
#include <Adafruit_BNO055.h>
//#define RC                   //use this when choosing what mode to be in and not signaled in. line 76
//#define DEBUG
//#define DEBUG_RC_IN

// #define MaxSpeed 65           // Max Speed in Percentage 
float MaxSpeed = 35; // Max RC Speed (out of 0-100)
float MaxPBSpeed = 30; // Max Playback Speed(out of 0-100)

#define Min_Steering 0        // SETTING MIN STEERING LIMITS // left
#define Mid_Steering 90       // SETTING MID STEERING LIMITS // stopped
#define Max_Steering 180      // SETTING MAX STEERING LIMITS // right
#define Min_Thrust 0          // SETTING MIN THRUST LIMITS // reverse
#define Mid_Thrust 90         // SETTING MID THRUST LIMITS // stopped
#define Max_Thrust 180        // SETTING MAX THRUST LIMITS // forward
#define ForwardThrustIn 1074   //what are these? DKF
#define BackwardThrustIn 1952  //max and min joystick values. IAS
#define RightDirIn 1935
#define LeftDirIn 1057

#define mode_All_STOP 3  //enumeration of modes
#define mode_RC 1
#define mode_RECORD 0
#define mode_PLAYBACK 2

#define DANGER_DIST 0              //ultrasonic distance for danger stop
#define CONCERN_DIST 500           //ultrasonic distance for concern
#define SLAVE_BROADCAST_ADDR 0x00  //default address for ultrasonic
#define SLAVE_ADDR 0x00            //SLAVE_ADDR 0xA0-0xAF for ultrasonic
#define NUMBER_OF_SENSORS 2
#define Number_of_Anchors 3

#define SteeringPin 27      //in to ESP32 from RC reciever
#define ThrustPin 35        //in to ESP32 from RC reciever
#define SpraySensPin 34     //in to ESP32 from RC reciever
#define SteeringSensPin 32  //in to ESP32 from RC reciever
#define RelayPin 25         //in to ESP32 from RC reciever
#define ResumePin 26        //in to ESP32 from RC reciever
#define ModePin 33          //in to ESP32 from RC reciever
#define rightSidePin 16     //out to motor controller
#define leftSidePin 17      //out to motor controller
#define DWM1000_RX 13        //in to ESP32 from DW1000
#define DWM1000_TX 14        //out to DW1000 from ESP32dd

#define OLED                 // to enable OLED screen
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define WhitePin 0     //i2c breakoutboard gpio 0
#define BluePin 1      //i2c breakoutboard gpio 1
#define GreenPin 2     //i2c breakoutboard gpio 2
#define OrangePin 3    //i2c breakoutboard gpio 3
#define RedPin 4       //i2c breakoutboard gpio 4
#define RelayOutPin 5  //i2c breakoutboard gpio 5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

TCA9534 ourGPIO;

QwiicUltrasonic LHUltraSonic;
QwiicUltrasonic RHUltraSonic;
//QwiicUltrasonic CEUltraSonic;

// #ifndef RC       //constant RC Mode for testing
// mode = mode_RC;
// #endif           //constant RC Mode for testing

#ifndef ESP32
OpenLog SD;
#endif

////////////////// Harware Interior TIMER INTERUPTs//////////////////////////
volatile int interruptCounter;  //for counting interrupt
int totalInterruptCounter;      //total interrupt counting
hw_timer_t* timer = NULL;       //H/W timer defining (Pointer to the Structure)
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimer() {  //Defining Inerrupt function with IRAM_ATTR for faster access
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
////////////////// Harware Interior TIMER INTERUPTs//////////////////////////

Servo leftSideServo;   // Servo Objects
Servo rightSideServo;  // Servo Objects

MovingAverage<long, 16> LHFilter;
MovingAverage<long, 16> RHFilter;
MovingAverage<long, 16> CEFilter;

uint8_t distance_H = 0;  //ultrasonic
uint8_t distance_L = 0;  //ultrasonic
uint16_t distance = 0;   //ultrasonic

volatile unsigned long pulseInTimeBegin_Steering = micros();  //Interrupt timers
volatile unsigned long pulseInTimeBegin_Thrust = micros();
volatile unsigned long pulseInTimeBegin_Relay = micros();
volatile unsigned long pulseInTimeBegin_Resume = micros();
volatile unsigned long pulseInTimeBegin_Mode = micros();
volatile unsigned long pulseInTimeBegin_ModeSens = micros();
volatile unsigned long pulseInTimeBegin_SteeringSens = micros();  //Interrupt timers

const byte numChars = 32;
const int numData = 9;
char receivedChars[numChars];
char tempChars[numChars], messageFromPC[numChars] = { 0 }, logMessage[numChars];
int integerFromPC = 0, anchor1 = 1780, anchor2 = 1781, anchor3 = 1782;
float floatFromPC = 0.0, anchorDistance1, anchorDistance2, anchorDistance3;
float xAnchors[Number_of_Anchors] = {0, 2.5, 0};  // X coordinates of anchors
float yAnchors[Number_of_Anchors] = {0, 8.5, 1};  // Y coordinates of anchors
float zAnchors[Number_of_Anchors] = {1.5, 1.5, 1};  // Z coordinates of anchors
double xPos, yPos, RecX, RecY;
float posTol = 1.0, headTol = 1.0;
bool newData = false, mode_Flag = false;
int incomingByte = 0;  // for incoming serial data
float LHMIX, RHMIX;
float Sc_MaxSteer, Sc_MinSteer, Sc_MaxThrust, Sc_MinThrust, SpeedCap; //scaled max n mins
int mode = 1;  //mode is record or playback
float Direction, Thrust;
volatile long SteeringDur = 1500, ThrustDur = 1500;
long RelayDur = 2000;
long SprayDuration;
long Spray_Sensitivity_Dur = 1500;
long Steering_Sensitivity_Dur = 1500;
long Mode_dur = 1000, ResumeDur = 1000;
int startTime;
int startTime_Playback;
long currentFilePosition = 0;
long RTime, dt;
int HeadingCurrent = 0;
int ultraSonic[] = { 0x22, 0x23 };  //used to check if ultra was connected or not
bool DangerFlag = false;
bool RelayFlag = false;
bool MadeitFlag = false;
bool EndOfFile = false;
bool SprayOutput = false;
bool recordFirstTime = false;
float data[numData];  //fitness is the root mean squared of how well the robot followed the path.
float errorTotal1 = 0, errorTotal2 = 0, errorTotal3 = 0;
float errorAnchor1 = 0, errorAnchor2 = 0, errorAnchor3 = 0;
float K_total, K_heading, K_location = 0.1;
float CurrentHeading = 0;
unsigned long previousMillis = 0;
int LEDTimer = 1000;
EasyBNO055_ESP bno;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  bno.start();  //starts up BNO IMU // without one hooked up this freezes the code
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1, DWM1000_RX, DWM1000_TX);  // UART Comm to DWM1000 UWB Modules
  Serial.println("serial setup");
  delay(1000);
  ourGPIO.begin();


#ifdef ESP32

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftSideServo.setPeriodHertz(50);                 // standard 50 hz servo
  leftSideServo.attach(leftSidePin, 1000, 2000);    // attaches the servo on pin 17 to the servo object
  rightSideServo.setPeriodHertz(50);                // standard 50 hz servo
  rightSideServo.attach(rightSidePin, 1000, 2000);  // attaches the servo on pin 16 to the servo object

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("Card Mounted baby!");
  }
#endif

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  pinMode(RelayPin, INPUT);
  pinMode(SteeringPin, INPUT);
  pinMode(ThrustPin, INPUT);
  pinMode(ModePin, INPUT);
  pinMode(SpraySensPin, INPUT);
  pinMode(SteeringSensPin, INPUT);
  ourGPIO.pinMode(RelayOutPin, GPIO_OUT);

  attachInterrupt(digitalPinToInterrupt(ModePin), Mode_CH5_PinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RelayPin), Spray_CH9_PinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteeringPin), Steering_CH1_PinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ThrustPin), Thrust_CH2_PinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ResumePin), Resume_CH10_PinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SpraySensPin), SpraySens_CH6_PinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteeringSensPin), SteeringSens_CH8_PinInterrupt, CHANGE);

  //LED PINS
  ourGPIO.pinMode(WhitePin, GPIO_OUT);   // WHITE
  ourGPIO.pinMode(BluePin, GPIO_OUT);    // BLUE
  ourGPIO.pinMode(GreenPin, GPIO_OUT);   // GREEN
  ourGPIO.pinMode(OrangePin, GPIO_OUT);  // ORANGE
  ourGPIO.pinMode(RedPin, GPIO_OUT);     // RED
  //LED PINS

  ////////////////////////// INTERIOR HARDWARE TIMER ////////////////////////////////////////////////
  startTime = millis();
  //timer = timerBegin(0, 80, true);              // timer 0, prescalar: 80, UP counting
  timer = timerBegin(100000);  // timer 0, prescalar: 80, UP counting
  //timerAttachInterrupt(timer, &onTimer, true);  // Attach interrupt
  timerAttachInterrupt(timer, &onTimer);  // Attach interrupt
  //timerAlarmWrite(timer, 100000, true);         // Match value= 1000000 for 1 sec. delay.
  timerAlarm(timer, 100000, true, 0);  // Match value= 1000000 for 1 sec. delay.
  //timerAlarmEnable(timer);                // Enable Timer with interrupt (Alarm Enable)
  ////////////////////////// INTERIOR HARDWARE TIMER ////////////////////////////////////////////////

  SpeedCap = (MaxSpeed / 100); //0-100 to 0-1
  Sc_MaxSteer = ((Max_Steering - Mid_Steering) * SpeedCap) + Mid_Steering; //scaled max n mins
  Sc_MinSteer = Mid_Steering - (Mid_Steering * SpeedCap); 

  Sc_MaxThrust = ((Max_Thrust - Mid_Thrust) * SpeedCap) + Mid_Thrust; //scaled max n mins
  Sc_MinThrust = Mid_Thrust - (Mid_Thrust * SpeedCap);
  
  SpeedCap = (MaxPBSpeed / 100); //0-100 to 0-1
  Max_PBSpeed = ((Max_Steering - Mid_Steering) * SpeedCap) + Mid_Steering; //scaled max n mins
  Min_PBSpeed = Mid_Steering - (Mid_Steering * SpeedCap); 
  Serial.print(String(" : Max_PBSpeed = ") + Max_PBSpeed + String(" : Min_PBSpeed = ") + Min_PBSpeed);

  delay(500);
  Lights(0,0,0,0,0,0); //all lights off
  delay(500);
}

//****************************MAIN LOOP**************************************************************************************************
//******************************************************************************************************************************************************
void loop() {

  GetLocalReadings();  //get UWB readings, for all modes

  if (mode == mode_RC) {  //mode RC
    Lights(0,0,1,0,0,0); // green on
    Spray();
    currentFilePosition = 0;  //resetting the file to the start
    EndOfFile = false;
    MIXRC();  //get RC PWM, for RC and REC only
    SprayDuration = RelayDur;
    startTime_Playback = millis();
    startTime = millis();
    recordFirstTime = true;
    data[0] = 0;  //set time to zero
  }

  if (mode == mode_RECORD) {  //mode record
    Lights(0,1,0,0,0,1);  // blue pulse

    if (recordFirstTime) {
      if (SD.remove("/log.txt")) {
        Serial.println("File deleted");
      } else {
        Serial.println("Delete failed");
      }
    }
    recordFirstTime = false;

    Spray();
    currentFilePosition = 0;  //resetting the file to the start
    MIXRC();                  //get RC PWM, for RC and REC only
    EndOfFile = false;
    SprayDuration = RelayDur;
    startTime_Playback = millis();
    data[0] = 0;  //set time to zero
  }

  if (mode == mode_PLAYBACK) {  //mode playback
    Lights(1,0,0,0,0,1); //white pulse
    //localEnvironmentCheck(); //check local sensors

    calculatePosition(); // get current position
    double prevXpos = xPos; // set as previous x position
    double prevYpos = yPos; // set as previous y position

    if (mode != mode_All_STOP) {
      readFile(SD, "/log.txt");                                                  // checks times and read next file line to get new instructions
      Serial.print(String("time stamp = ") + data[0]);
      RecX = data[3]; //store x position to head to
      RecY = data[4]; //store y position to head to
      SprayOutput = data[5];                                                     // setting spray trigger to what came in from playback file
      Serial.print(String(" : WhereIam (x,y) = (") + prevXpos + String(",") + prevYpos + String(")"));
      Serial.print(String(" : WhereIgo (x,y) = (") + RecX + String(",") + RecY + String(")"));
      bresenhamLine(prevXpos, prevYpos, RecX, RecY); // apply bresenhams line algorithm
      Serial.print(" !after bresenham! ");

      prevXpos = RecX;
      prevYpos = RecY;

      // Serial.print(String(" : errAnch1 = ") + errorAnchor1 + String(" : errAnch2 = ") + errorAnchor2);
      // Serial.print(String(" : LHMIX_Rec = ") + data[1] + String(" : RHMIX_Rec = ") + data[2]);
      Serial.println();
    }
    if (EndOfFile) {
      Serial.println("end of file");
      AllStop_FXN();
      currentFilePosition = 0;
    }
  }

  ////////////////////// HARWARE INTERIOR TIMER FOR RECORDING SAMPLING////////////////
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    if (mode == mode_RECORD) {
      recordPath(SD);
    }
    totalInterruptCounter++;  //counting total interrupt
  }
  ////////////////////// HARWARE INTERIOR TIMER FOR RECORDING SAMPLING////////////////

  checkMode();
  Output();

  #ifdef DEBUG
  debugDisplay();
  #endif

  #ifdef DEBUG_RC_IN
  debugRC_IN();
  #endif

  //#ifdef OLED
  display.clearDisplay();
  OLED_display();
  //#endif
}
//********************************************************************************************************************************************
void GetLocalReadings() {  // one of the subroutines that reads the serial from the DW1000 location boards
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
  }
}

void recvWithStartEndMarkers() {  // one of the subroutines that reads the serial from the DW1000 location boards
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {                      // split the data into its parts
  char* strtokIndx;                     // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ":");  // this continues where the previous call left off
  strcpy(messageFromPC, strtokIndx);
  integerFromPC = atoi(strtokIndx);  // convert this part (ascii code) to an integer
  strtokIndx = strtok(NULL, ":");
  floatFromPC = atof(strtokIndx);  // convert this part to a float
  if (integerFromPC == anchor1) anchorDistance1 = floatFromPC;
  if (integerFromPC == anchor2) anchorDistance2 = floatFromPC;
  //if (integerFromPC == anchor3) anchorDistance3 = floatFromPC;
  calculatePosition();
}

// Function to calculate robot position using trilateration
void calculatePosition() {
  double A = 2 * (xAnchors[1] - xAnchors[0]);
  double B = 2 * (yAnchors[1] - yAnchors[0]);
  double C = 2 * (zAnchors[1] - zAnchors[0]);
  double D = pow(anchorDistance1, 2) - pow(anchorDistance2, 2) - pow(xAnchors[0], 2) + pow(xAnchors[1], 2) - pow(yAnchors[0], 2) + pow(yAnchors[1], 2) - pow(zAnchors[0], 2) + pow(zAnchors[1], 2);

  double E = 2 * (xAnchors[2] - xAnchors[0]);
  double F = 2 * (yAnchors[2] - yAnchors[0]);
  double G = 2 * (zAnchors[2] - zAnchors[0]);
  double H = pow(anchorDistance1, 2) - pow(anchorDistance3, 2) - pow(xAnchors[0], 2) + pow(xAnchors[2], 2) - pow(yAnchors[0], 2) + pow(yAnchors[2], 2) - pow(zAnchors[0], 2) + pow(zAnchors[2], 2);

  xPos = (D * F - H * B) / (A * F - E * B);
  yPos = (D * E - A * H) / (B * E - A * F);
}

void bresenhamLine(float x1, float y1, float x2, float y2) {
  // The result should be intermediate x,y points that the robot should follow
  MadeitFlag = false;
  int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  Serial.print(String(" : dx = ") + dx + String(" : dy = ") + dy);
  int err = dx + dy, e2;  // error value e_xy

  if(MadeitFlag == false && mode == mode_PLAYBACK && mode != mode_All_STOP) {
    CurrentHeading = bno.orientationZ;
    float targetHeading = calculateHeadingToTarget(x2, y2);  // Calculate desired heading to the target point
    float headingError = targetHeading - CurrentHeading;   // Current heading error    
    Serial.print(String(" : targetHeading = ") + targetHeading + String(" : CurrentHeading = ") + CurrentHeading + String(" : headingError = ") + headingError);

    moveRobotTo(dx, dy, headingError);  // Move the robot to the next point (x2, y2)
    if (abs(dx) <= posTol && abs(dy) <= posTol && abs(headingError) < headTol) {
      MadeitFlag = true;
      Serial.print(" !made it! ");
    }
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x2 += sx; }
    if (e2 <= dx) { err += dx; y2 += sy; }
  }
}

float calculateHeadingToTarget(int x, int y) {
  // Calculate the angle between the current position and the target (x, y)
  calculatePosition();
  double currentX = xPos;
  double currentY = yPos;
  float deltaX = x - currentX;
  float deltaY = y - currentY;
  //Serial.print(String(" : deltaX = ") + deltaX + String(" : deltaY = ") + deltaY);
  return atan2(deltaY, deltaX) * (180.0 / PI);  // Convert to degrees
}

void moveRobotTo(float x,float y,float headingError) {
  LHMIX = Max_PBSpeed; //setting max playback speed
  RHMIX = Max_PBSpeed; //setting max playback speed

  if (dx > 1 || dy > 1){  //while moving, adjust heading
    if (headingError < 0){
      LHMIX = LHMIX - (headingError * K_heading); //turn left a bit
    }
    else if (headingError > 0){
      RHMIX = RHMIX - (headingError * K_heading); //turn right a bit
    }
  }

  else if (dx < 1 || dy < 1){ //at position, just needs to rotate
    if (headingError < 0){ //rotate left
      LHMIX = Min_PBSpeed;
      RHMIX = Max_PBSpeed;
    }
    else if (headingError > 0){ //rotate right
      LHMIX = Max_PBSpeed;
      RHMIX = Min_PBSpeed;
    }
  }
  Serial.print(String(" : LHMIX = ") + LHMIX + String(" : RHMIX = ") + RHMIX);
}

void MIXRC() {  //pulse in for RC PWM and filters it to output LHMIX,RHMIX

  Direction = (SteeringDur - LeftDirIn) * (Sc_MaxSteer - Sc_MinSteer);
  Direction = Direction / (RightDirIn - LeftDirIn);
  Direction = Direction + Sc_MinSteer;

  Thrust = (ThrustDur - BackwardThrustIn) * (Sc_MaxThrust - Sc_MinThrust);
  Thrust = Thrust / (ForwardThrustIn - BackwardThrustIn);
  Thrust = Thrust + Sc_MinThrust;

  if (Direction > Sc_MaxSteer) Direction = Sc_MaxSteer;  //setting caps
  if (Direction < Sc_MinSteer) Direction = Sc_MinSteer;
  if (Thrust > Sc_MaxThrust) Thrust = Sc_MaxThrust;
  if (Thrust < Sc_MinThrust) Thrust = Sc_MinThrust;

  LHMIX = Thrust + (Direction - 90) * (Steering_Sensitivity_Dur * .05) / 100;  //doing a little DJ Dan and the fresh tunes mixing
  RHMIX = Thrust - (Direction - 90) * (Steering_Sensitivity_Dur * .05) / 100;
}

void Spray() {
  if ((SprayDuration < 1500) && (RelayFlag != 1)) {  //this causes problems when there is playback!!!!
    //turn on relay
    RTime = micros();
    RelayFlag = 1;  // flag to say relay is on
    SprayOutput = true;
    ourGPIO.digitalWrite(RelayOutPin, LOW);
  }

  if ((micros() - RTime) > (Spray_Sensitivity_Dur * 6920 - 5390560)) {
    SprayOutput = false;
    ourGPIO.digitalWrite(RelayOutPin, HIGH);  //turn relay off
  }

  if (SprayDuration > 1500) RelayFlag = 0;  //turning relay off flag only when dur is reset

  if ((micros() - RTime) > 10000000) RTime = micros() + 1100000;  //reset RTIME to keep in in bounds
}

void recordPath(fs::FS& fil) {

  long tim = (millis() - startTime);  //make an int for time
  File skunkLog = fil.open("/log.txt", FILE_APPEND);

  if (!skunkLog) {
    Serial.println("Failed to APPEND file for writing");
  } else {

  }                     // Determine the total length needed for the concatenated strin
  calculatePosition();
  skunkLog.print(tim);  //data[0]
  skunkLog.print(";");
  skunkLog.print(LHMIX);  //data[1]
  skunkLog.print(";");
  skunkLog.print(RHMIX);  //data[2]
  skunkLog.print(";");
  skunkLog.print(xPos);  //data[3]
  skunkLog.print(";");
  skunkLog.print(yPos);  //data[4]
  skunkLog.print(";");
  skunkLog.print(SprayOutput);  //data[5]
  skunkLog.print(";");
  skunkLog.print(anchorDistance1);  //data[6]
  skunkLog.print(";");
  skunkLog.print(anchorDistance2);  //data[7]
  skunkLog.print(";");
  skunkLog.print(anchorDistance3);  //data[8]
  skunkLog.print(";");
  skunkLog.print(bno.orientationZ);  //data[9]

  Serial.print(String("time = ") + tim + String(" : LHMIX = ") + LHMIX + String(" : RHMIX = ") + RHMIX + String(" : orientationZ = ") + bno.orientationZ);
  Serial.println(String(" : xPos = ") + xPos + String(" : yPos = ") + yPos + String(" : anch1 = ") + anchorDistance1 + String(" : anch2 = ") + anchorDistance2 + String(" : anch3 = ") + anchorDistance3);

  skunkLog.println(";"); //end with a ;
  skunkLog.close();
}

void readFile(fs::FS& fs, const char* path) { //where can i add something to wait to move to next line
  //Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  String line = file.readStringUntil('\n');
  file.seek(currentFilePosition);
  bool lineDone = false;
  int i = 0;
  char value[7];
  memset(value, '\0', sizeof(value));  //clear out array

  if (!file.available()) {
    Serial.println("file unavailable");
    AllStop_FXN();
    EndOfFile = true;
  }
  while (file.available() && !lineDone) {
    char newData = file.read();
    if (newData == ';') {  //done reading in value data point
      data[i] = atof(value);
      i++;
      memset(value, '\0', sizeof(value));  //clear out array

    } else if (newData == '\n' && MadeitFlag) {          //done with line of file
      i = 0;
      lineDone = true;
      currentFilePosition = file.position();

    } else {  //add new character to value array
      int length = strlen(value);
      value[length] = newData;
    }
  }
}

void debugDisplay() {  //only for debug serial monitor
  Serial.print("anchor dist = ");
  Serial.print(" =");
  Serial.print(anchorDistance1);
  Serial.print(" ThrustDur = ");
  Serial.print(anchorDistance2);
  Serial.print("=");
  Serial.print(anchorDistance2);
  Serial.print("");
  Serial.print(" HEADING = ");
  Serial.print(bno.orientationZ);
  Serial.print(" mode = ");
  if (mode == mode_RECORD) { Serial.print("RECORDING"); }
  if (mode == mode_PLAYBACK) { Serial.print("PLAYBACK"); }
  if (mode == mode_RC) { Serial.print("RC"); }
  if (mode == mode_All_STOP) { Serial.print("ALL STOP: SHUT IT DOWN EARL!"); }
  Serial.println("");
}

void debugRC_IN() {
  Serial.print("dt = ");
  Serial.print(dt);
  Serial.print(": THRUST = ");
  Serial.print(ThrustDur);
  Serial.print(": STEER = ");
  Serial.print(SteeringDur);
  Serial.print(": LMIX = ");
  Serial.print(LHMIX);
  Serial.print(": RHMIX = ");
  Serial.print(RHMIX);
  Serial.print(" Steering =");
  Serial.print((Steering_Sensitivity_Dur * .09 - 80) / 100);
  Serial.print(": MODE = ");
  if (mode == mode_RECORD) { Serial.print("RECORDING"); }
  if (mode == mode_PLAYBACK) { Serial.print("PLAYBACK"); }
  if (mode == mode_RC) { Serial.print("RC"); }
  if (mode == mode_All_STOP) { Serial.print("ALL STOP: SHUT IT DOWN EARL!"); }
  Serial.println("");
}

void localEnvironmentCheck() {  //ultrasonic sensors on a I2C MUX WARNING:Wonder if this whole process is expensive?
  uint16_t LHdistance = 0;
  uint16_t RHdistance = 0;
  uint16_t CEdistance = 0;
  LHUltraSonic.triggerAndRead(LHdistance);
  RHUltraSonic.triggerAndRead(RHdistance);
  //CEUltraSonic.triggerAndRead(CEdistance);
  LHFilter.add(LHdistance);
  RHFilter.add(RHdistance);
  //CEFilter.add(CEdistance);
}

void checkMode() {

  if ((LHFilter.get() < DANGER_DIST) || (RHFilter.get() < DANGER_DIST) || (CEFilter.get() < DANGER_DIST)) {  //CHECKING FOR DANGER CLOSE ULTRASONIC
    DangerFlag = true;
    AllStop_FXN(); //change to pause function so if something moves out of way it can resume/ 
    Serial.println("DANGER CLOSE");

  } else if ((Mode_dur < 1800) && (Mode_dur > 1200))  //mode record is turned on
  {
#ifndef RC
    mode = mode_RC;
    mode_Flag = 0;
#endif
    //mode_Flag = 1;
  } else if (Mode_dur > 1800)  // mode RC is turned on
  {
#ifndef RC
    //set LED to RC
    mode = mode_RECORD;
#endif
  } else if ((Mode_dur < 1200) && (mode != mode_All_STOP))  // mode playback is turned on and not coming from all stop
  {
#ifndef RC
    mode = mode_PLAYBACK;
    mode_Flag = 0;
#endif
  } else {
#ifndef RC
    mode = mode_All_STOP;
#endif
  }  // put in 1-19-24 to get rid of residual turning after all stop
}

void OLED_display() {
  // Serial.println("made it");
  display.setTextSize(1);  // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.print(LHFilter.get());
  display.setCursor(50, 0);
  display.print(CEFilter.get());
  display.setCursor(90, 0);
  display.print(RHFilter.get());
  display.setCursor(0, 20);
  display.print("MODE= ");
  switch (mode) {
    case mode_RECORD:
      display.print("RECORDING");
      break;
    case mode_RC:
      display.print("RC = ");
      display.print(sqrt(errorTotal1) + sqrt(errorTotal2));
      break;
    case mode_PLAYBACK:
      display.print("PLAYBACK");
      break;
    case mode_All_STOP:
      display.print("ALL STOP");
      break;
  }
  display.display();  // Show initial text
  //Serial.println("done");
}

void Output() {
  // Serial.println(String("LHMIX = ") + LHMIX + String(" : RHMIX = ") + RHMIX);
  rightSideServo.write(RHMIX);  // tell servo to go to position in variable 'pos'
  leftSideServo.write(LHMIX);

  if (SprayOutput) {  //This is where the spray relay gets turned on or off, multiple modes affect this
    ourGPIO.digitalWrite(RelayOutPin, LOW);
  } else {
    ourGPIO.digitalWrite(RelayOutPin, HIGH);
  }
}

void AllStop_FXN() {
  Lights(0,0,0,0,1,0);
  mode = mode_All_STOP;
  rightSideServo.write(90);   // motors stop
  leftSideServo.write(90);    // motors stop
  ourGPIO.digitalWrite(RelayOutPin, HIGH);
}

void Lights(bool White, bool Blue, bool Green, bool Orange, bool Red, bool Pulse) { // lights function
  unsigned long currentMillis = millis();
  //Serial.println(); //only works if this is here
  // Turn off all LEDs that are not specified
  if (!White) ourGPIO.digitalWrite(WhitePin, HIGH);
  if (!Blue) ourGPIO.digitalWrite(BluePin, HIGH);
  if (!Green) ourGPIO.digitalWrite(GreenPin, HIGH);
  if (!Orange) ourGPIO.digitalWrite(OrangePin, HIGH);
  if (!Red) ourGPIO.digitalWrite(RedPin, HIGH);

  if (Pulse) {
    if (currentMillis - previousMillis >= LEDTimer) {
      previousMillis = currentMillis;
      
      // Toggle the LEDs based on the input
      if (White && ourGPIO.digitalRead(WhitePin) == HIGH) {
        ourGPIO.digitalWrite(WhitePin, LOW);
      } else if (White) {
        ourGPIO.digitalWrite(WhitePin, HIGH);
      }
      if (Blue && ourGPIO.digitalRead(BluePin) == HIGH) {
        ourGPIO.digitalWrite(BluePin, LOW);
      } else if (Blue) {
        ourGPIO.digitalWrite(BluePin, HIGH);
      }
      if (Green && ourGPIO.digitalRead(GreenPin) == HIGH) {
        ourGPIO.digitalWrite(GreenPin, LOW);
      } else if (Green) {
        ourGPIO.digitalWrite(GreenPin, HIGH);
      }
      if (Orange && ourGPIO.digitalRead(OrangePin) == HIGH) {
        ourGPIO.digitalWrite(OrangePin, LOW);
      } else if (Orange) {
        ourGPIO.digitalWrite(OrangePin, HIGH);
      }
      if (Red && ourGPIO.digitalRead(RedPin) == HIGH) {
        ourGPIO.digitalWrite(RedPin, LOW);
      } else if (Red) {
        ourGPIO.digitalWrite(RedPin, HIGH);
      }
    }
  } else {
    // Turn on the LEDs without pulsing
    if (White) ourGPIO.digitalWrite(WhitePin, LOW);
    if (Blue) ourGPIO.digitalWrite(BluePin, LOW);
    if (Green) ourGPIO.digitalWrite(GreenPin, LOW);
    if (Orange) ourGPIO.digitalWrite(OrangePin, LOW);
    if (Red) ourGPIO.digitalWrite(RedPin, LOW);
  }
}

////////////////////////////////////////////////INTERRUPT PINS////////////////////////////////////////////////
void Steering_CH1_PinInterrupt() {  //STEERING INTERRUPT PIN
  if ((digitalRead(SteeringPin) == HIGH)) {
    pulseInTimeBegin_Steering = micros();
  } else {
    SteeringDur = micros() - pulseInTimeBegin_Steering;  // stop measuring
  }
}
void Thrust_CH2_PinInterrupt() {  //THRUST INTERRUPT PIN
  if ((digitalRead(ThrustPin) == HIGH)) {
    pulseInTimeBegin_Thrust = micros();
  } else {
    ThrustDur = micros() - pulseInTimeBegin_Thrust;
  }
}
void Spray_CH9_PinInterrupt() {  //SPRAY INTERRUPT PIN
  if ((digitalRead(RelayPin)) == HIGH) {
    pulseInTimeBegin_Relay = micros();
  } else {
    RelayDur = micros() - pulseInTimeBegin_Relay;
  }
}
void Resume_CH10_PinInterrupt() {  //RESUME INTERRUPT PIN
  if ((digitalRead(ResumePin)) == HIGH) {
    pulseInTimeBegin_Resume = micros();
  } else {
    ResumeDur = micros() - pulseInTimeBegin_Resume;
  }
}
void Mode_CH5_PinInterrupt() {  //MODE INTERRUPT PIN
  if ((digitalRead(ModePin)) == HIGH) {
    pulseInTimeBegin_Mode = micros();
  } else {
    Mode_dur = micros() - pulseInTimeBegin_Mode;
  }
}
void SpraySens_CH6_PinInterrupt() {  //SPRAY SENSITIVITY INTERRUPT PIN
  if ((digitalRead(SpraySensPin)) == HIGH) {
    pulseInTimeBegin_ModeSens = micros();
  } else {
    Spray_Sensitivity_Dur = micros() - pulseInTimeBegin_ModeSens;
  }
}
void SteeringSens_CH8_PinInterrupt() {  //STEERING SNESITIVITY INTERRUPT PIN
  if ((digitalRead(SteeringSensPin)) == HIGH) {
    pulseInTimeBegin_SteeringSens = micros();
  } else {
    Steering_Sensitivity_Dur = micros() - pulseInTimeBegin_SteeringSens;
  }
}
////////////////////////////////////////////////INTERRUPT PINS////////////////////////////////////////////////
