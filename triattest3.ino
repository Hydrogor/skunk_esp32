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

#define ForwardThrustIn 1074  // min of y joystick values
#define BackwardThrustIn 1952 // max of y joystick values
#define RightDirIn 1935       // min of x joystick values
#define LeftDirIn 1057        // max of x joystick values

#define mode_All_STOP 3  //enumeration of modes
#define mode_RC 1
#define mode_RECORD 0
#define mode_PLAYBACK 2
#define mode_PAUSE 4
#define mode_RESUME 5

#define DANGER_DIST 0              //ultrasonic distance for danger stop
#define CONCERN_DIST 500           //ultrasonic distance for concern
#define SLAVE_BROADCAST_ADDR 0x00  //default address for ultrasonic
#define SLAVE_ADDR 0x00            //SLAVE_ADDR 0xA0-0xAF for ultrasonic
#define Number_of_UltraSensor 2
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
volatile unsigned long pulseInTimeBegin_SpraySens = micros();
volatile unsigned long pulseInTimeBegin_SteeringSens = micros();  //Interrupt timers

const byte numChars = 32;
const int numData = 10;
char receivedChars[numChars];
char tempChars[numChars], messageFromPC[numChars] = { 0 }, logMessage[numChars];
int integerFromPC = 0, anchor1 = 1780, anchor2 = 1781, anchor3 = 1782;
float floatFromPC = 0.0, anchDist1, anchDist2, anchDist3;
float xAnchors[Number_of_Anchors] = {0, 2.5, 0};  // X coordinates of anchors
float yAnchors[Number_of_Anchors] = {0, 8.5, 1};  // Y coordinates of anchors
float zAnchors[Number_of_Anchors] = {1.5, 1.5, 1};  // Z coordinates of anchors
double xPos, yPos, RecX, RecY;
float posTol = 1.0, headTol = 5.0; //playback tolerances
bool newData = false, mode_Flag = false, lineDone = false;
int incomingByte = 0;  // for incoming serial data
float LHMIX, RHMIX;
float Sc_MaxSteer, Sc_MinSteer, Sc_MaxThrust, Sc_MinThrust, Max_PBSpeed, Min_PBSpeed, SpeedCap; //scaled max n mins
int mode = 1;  //mode is record or playback
float Direction, Thrust;
volatile long SteeringDur = 1500, ThrustDur = 1500;
long RelayDur = 2000, SprayDur = 0, RecOrient = 0, TimeStamp = 0;
float targetHeading = 0, headingError = 0;
long SprayDuration;
long Spray_Sensitivity_Dur = 1500;
long Steering_Sensitivity_Dur = 1500;
long Mode_dur = 1000, ResumeDur = 1000;
int startTime;
int startTime_Playback;
long currentFilePosition = 0;
long RTime, dt;
int ultraSonic[] = { 0x22, 0x23 };  //used to check if ultra was connected or not
bool DangerFlag = false;
bool RelayFlag = false;
bool MadeitFlag = false;
bool EndOfFile = false;
bool SprayOutput = false;
bool recordFirstTime = false;
bool MoveFlag = false, RotateFlag = false;
float data[numData];  //fitness is the root mean squared of how well the robot followed the path.
float errTot = 0, errTotDev = 0;
float errAnch1 = 0, errAnch2 = 0, errAnch3 = 0;
float K_total, K_heading, K_location = 0.1;
float CurrentHeading = 0, OrientError = 0;
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
  timer = timerBegin(100000);  // 1000000hz 
  timerAttachInterrupt(timer, &onTimer);  // Attach interrupt
  timerAlarm(timer, 25000, true, 0);  // Argument2 = 1000000 for 1 sec / 250000 = .25sec [timestamp interval]
  ////////////////////////// INTERIOR HARDWARE TIMER ////////////////////////////////////////////////

  SpeedCap = (MaxSpeed / 100); //0-100 to 0-1
  Sc_MaxSteer = ((Max_Steering - Mid_Steering) * SpeedCap) + Mid_Steering; //scaled max n mins
  Sc_MinSteer = Mid_Steering - (Mid_Steering * SpeedCap); 

  Sc_MaxThrust = ((Max_Thrust - Mid_Thrust) * SpeedCap) + Mid_Thrust; //scaled max n mins
  Sc_MinThrust = Mid_Thrust - (Mid_Thrust * SpeedCap);
  
  SpeedCap = (MaxPBSpeed / 100); //0-100 to 0-1
  Max_PBSpeed = ((Max_Steering - Mid_Steering) * SpeedCap) + Mid_Steering; //scaled max n mins
  Min_PBSpeed = Mid_Steering - (Mid_Steering * SpeedCap); 

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
    errTot = 0;
    calculatePosition(); // get current position
    double prevXpos = xPos; // set as previous x position
    double prevYpos = yPos; // set as previous y position
    startTime_Playback = millis();                                   // i only want to do this once
    //Serial.println(String("startTime = ") + startTime_Playback);

    if (mode != mode_All_STOP) {
      readFile(SD, "/log.txt");                                                  // checks times and read next file line to get new instructions
      TimeStamp = data[0];
      RecX = data[1]; //store x position to head to
      RecY = data[2]; //store y position to head to
      errAnch1 = (data[3] - anchDist1) * (data[3] - anchDist1);
      errAnch2 = (data[4] - anchDist2) * (data[4] - anchDist2);
      errAnch3 = (data[5] - anchDist3) * (data[5] - anchDist3);
      RecOrient = data[6];
      SprayOutput = data[7];                                                     // setting spray trigger to what came in from playback file
      SprayDur = data[8];

      errTotDev = errAnch1 + errAnch2 + errAnch1; // error at each timestamp
      errTot = errTot + errAnch1 + errAnch2 + errAnch3; // total error
      bresenhamLine(prevXpos, prevYpos, RecX, RecY); // apply bresenhams line algorithm
      if (MadeitFlag){
        recordPath(SD, "skunkPBLog"); //after it gets to pos, record data
        prevXpos = RecX;
        prevYpos = RecY;
        MadeitFlag = false;

        if (DangerFlag) {
          //records where it is in file
          //then when resumed it can return to file
        }
    
        if (EndOfFile) {
          Serial.println(String("!!End of file!! : Total Error = ") + errTot + String(" : Playback Time = ") + (millis()-startTime_Playback) + String("ms"));// playback time resests every loop
          AllStop_FXN();
          currentFilePosition = 0;
        }
      }
    }
  }

  ////////////////////// HARWARE INTERIOR TIMER FOR RECORDING SAMPLING////////////////
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    if (mode == mode_RECORD) {
      recordPath(SD, "skunkLog");
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
  if (integerFromPC == anchor1) anchDist1 = floatFromPC;
  if (integerFromPC == anchor2) anchDist2 = floatFromPC;
  //if (integerFromPC == anchor3) anchDist3 = floatFromPC;
  calculatePosition();
}

// Function to calculate robot position using trilateration
void calculatePosition() {
  double A = 2 * (xAnchors[1] - xAnchors[0]);
  double B = 2 * (yAnchors[1] - yAnchors[0]);
  double C = 2 * (zAnchors[1] - zAnchors[0]);
  double D = pow(anchDist1, 2) - pow(anchDist2, 2) - pow(xAnchors[0], 2) + pow(xAnchors[1], 2) - pow(yAnchors[0], 2) + pow(yAnchors[1], 2) - pow(zAnchors[0], 2) + pow(zAnchors[1], 2);

  double E = 2 * (xAnchors[2] - xAnchors[0]);
  double F = 2 * (yAnchors[2] - yAnchors[0]);
  double G = 2 * (zAnchors[2] - zAnchors[0]);
  double H = pow(anchDist1, 2) - pow(anchDist3, 2) - pow(xAnchors[0], 2) + pow(xAnchors[2], 2) - pow(yAnchors[0], 2) + pow(yAnchors[2], 2) - pow(zAnchors[0], 2) + pow(zAnchors[2], 2);

  xPos = (D * F - H * B) / (A * F - E * B);
  yPos = (D * E - A * H) / (B * E - A * F);
}

void bresenhamLine(float x1, float y1, float x2, float y2) {
  // The result should be intermediate x,y points that the robot should follow
  MadeitFlag = false;
  float dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  float dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  //Serial.print(String(" : dx = ") + dx + String(" : dy = ") + dy);
  float err = dx + dy, e2;  // error value e_xy

  if (MadeitFlag == false && mode == mode_PLAYBACK) {//might need to make a while loop to keep moving until it actually gets to pos
    moveRobotTo(x2, y2);  // Move the robot to the next point (x2, y2)

    if (abs(dx) <= posTol && abs(dy) <= posTol){

      if (MoveFlag && abs(headingError) < headTol){
        MadeitFlag = true;
        MoveFlag = false;
        Serial.println(" : !made it - moved! ");
      }

      if (RotateFlag && abs(OrientError) < headTol){
        MadeitFlag = true;
        RotateFlag = false;
        Serial.println(" : !made it - rotated! ");
      }
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

void moveRobotTo(float x2, float y2) {
  LHMIX = Max_PBSpeed;  //setting max playback speed
  RHMIX = Max_PBSpeed;  //setting max playback speed
  calculatePosition();
  double currentX = xPos;
  double currentY = yPos;
  float dx = abs(x2 - currentX), sx = currentX < x2 ? 1 : -1;
  float dy = -abs(y2 - currentY), sy = currentY < y2 ? 1 : -1;
  float err = dx + dy, e2;  // error value e_xy

  if (dx > 1 || dy > 1) {  //while moving, adjust heading
    CurrentHeading = bno.orientationZ;
    targetHeading = calculateHeadingToTarget(x2, y2);  // Calculate desired heading to the target point
    float headingError = targetHeading - CurrentHeading;   // Current heading error  
    MoveFlag = true;
    //Serial.print(String(" : targetHeading = ") + targetHeading + String(" : CurrentHeading = ") + CurrentHeading + String(" : headingError = ") + headingError);

    if (headingError < 0) {
      LHMIX = LHMIX - (headingError * K_heading);  //turn left a bit
    } else if (headingError > 0) {
      RHMIX = RHMIX - (headingError * K_heading);  //turn right a bit
    }
  }

  if (dx < 1 && dy < 1) {  //at position, just needs to rotate
    RotateFlag = true;
    CurrentHeading = bno.orientationZ;
    OrientError = RecOrient - CurrentHeading;
    //Serial.print(String(" : RecOrient = ") + RecOrient + String(" : CurrentHeading = ") + CurrentHeading + String(" : OrientError = ") + OrientError);

    if (OrientError < 0) {  //rotate left
      LHMIX = Min_PBSpeed;
      RHMIX = Max_PBSpeed;
    } else if (OrientError > 0) {  //rotate right
      LHMIX = Max_PBSpeed;
      RHMIX = Min_PBSpeed;
    }
  }
  e2 = 2 * err;
  if (e2 >= dy) { err += dy; x2 += sx; }
  if (e2 <= dx) { err += dx; y2 += sy; }
  if (mode == mode_RC) {break;}
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
  SprayDur = Spray_Sensitivity_Dur * 6920 - 5390560;  //SprayDur = how long to spray (variable by knob)
  if ((SprayDuration < 1500) && (RelayFlag == false)) {  //this causes problems when there is playback!!!!
    //turn on relay
    RTime = micros();
    RelayFlag = true;  // flag to say relay is on
    SprayOutput = true;
    ourGPIO.digitalWrite(RelayOutPin, LOW); // turn relay on
  }

  if ((micros() - RTime) > SprayDur) {
    SprayOutput = false;
    ourGPIO.digitalWrite(RelayOutPin, HIGH);  //turn relay off after 2-8 sec
  }

  if (SprayDuration > 1500) RelayFlag = false;  //turning relay off flag only when dur is reset
  if ((micros() - RTime) > 10000000) RTime = micros() + 1100000;  //reset RTIME to keep in in bounds
}

void recordPath(fs::FS& fil, String filename) {
  float temptim = 0;
  File logFile;
  long tim = (millis() - startTime);  //make an int for time
  
  if (filename == "skunkLog") {
    logFile = fil.open("/log.txt", FILE_APPEND);       //recorded data
  }
  if (filename == "skunkPBLog") {
    logFile = fil.open("/logPB.txt", FILE_APPEND);   //playback data
    //maybe implement multiple playback files for testing? add counter which creates new files; if new record file, set counter to zero
  }
  if (!logFile) Serial.println("Failed to APPEND file for writing");
  
  calculatePosition();
  
  if (filename == "skunkLog") {
    logFile.print(tim);                              //data[0] - timestamp
    logFile.print(";");                          
    logFile.print(xPos);                             //data[1] - X position of robot
    logFile.print(";");                  
    logFile.print(yPos);                             //data[2] - Y position of robot
    logFile.print(";");              
    logFile.print(anchDist1);                        //data[3] - Anchor 1 Distance
    logFile.print(";");              
    logFile.print(anchDist2);                        //data[4] - Anchor 2 Distance
    logFile.print(";");              
    logFile.print(anchDist3);                        //data[5] - Anchor 3 Distance
    logFile.print(";");              
    logFile.print(bno.orientationZ);                 //data[6] - Orientation
    logFile.print(";");              
    logFile.print(SprayOutput);                      //data[7] - Spray Flag
    logFile.print(";");
    if (SprayOutput) logFile.print(SprayDur);        //data[8] - if spray Flag is true, output RelayDur length
    if (!SprayOutput) logFile.print(SprayOutput);    //data[8] - if spray Flag is false, output zero

    Serial.print(String("Time Stamp = ") + String(float(tim/1000.0), 2) + String("s"));       //recording time
    Serial.print(String(" : Position (x,y) = (") + xPos + String(",") + yPos + String(")m")); //position data
    Serial.print(String(" : Orient = ") + bno.orientationZ);                            //orientation data
    Serial.print(String(" : Anchor Dist's (1,2,3) = (") + anchDist1 + String(", ") + anchDist2 + String(", ") + anchDist3 + String(")m")); //anch Distance data
    Serial.print(" : SprayFlag = "); //spray out put on/off
    if (SprayOutput){ 
      Serial.print("On");
      Serial.print(String(" : SprayDur = ") + String(float(SprayDur/1000000.0), 2)); //spray duration in seconds
    } else {
      Serial.print("Off");
      Serial.print(String(" : SprayDur = ") + SprayOutput); // spray duration = 0 bc off
    }
  }

  if (filename == "skunkPBLog") {
    logFile.print(tim);                              //data[0] - timestamp
    logFile.print(";");                          
    logFile.print(xPos);                             //data[1] - X position of robot
    logFile.print(";");                  
    logFile.print(yPos);                             //data[2] - Y position of robot
    logFile.print(";");              
    logFile.print(anchDist1);                        //data[3] - Anchor 1 Distance
    logFile.print(";");              
    logFile.print(anchDist2);                        //data[4] - Anchor 2 Distance
    logFile.print(";");              
    logFile.print(anchDist3);                        //data[5] - Anchor 3 Distance
    logFile.print(";");              
    logFile.print(bno.orientationZ);                 //data[6] - Orientation
    logFile.print(";");              
    logFile.print(SprayOutput);                      //data[7] - Spray Flag
    logFile.print(";");
    if (SprayOutput) logFile.print(SprayDur);        //data[8] - if spray Flag is true, output RelayDur length
    if (!SprayOutput) logFile.print(SprayOutput);    //data[8] - if spray Flag is false, output zero
    logFile.print(";");
    logFile.print(TimeStamp);                        //data[9] - recorded timestamp into playback data used to compare with recorded timestamps
    logFile.print(";");
    logFile.print(errTotDev);                        //data[10] - position error
  
    Serial.print(String("PBTime Stamp = ") + String(float(tim/1000.0), 2) + String("s"));
    Serial.print(String(" : RecTime Stamp = ") + String(float(TimeStamp/1000.0), 2) + String("s"));
    Serial.print(String(" : Position (x,y) = (") + xPos + String(",") + yPos + String(")m")); //position data
    Serial.print(String(" : Orient = ") + bno.orientationZ);                            //orientation data
    Serial.print(String(" : Anchor Dist's (1,2,3) = (") + anchDist1 + String(", ") + anchDist2 + String(", ") + anchDist3 + String(")m")); //anch Distance data
    Serial.print(String(" : Error = ") + errTotDev);
    Serial.print(" : SprayFlag = "); //spray out put on/off
    if (SprayOutput){ 
      Serial.print("On");
      Serial.print(String(" : SprayDur = ") + String(float(SprayDur/1000000.0), 2)); //spray duration in seconds
    } else {
      Serial.print("Off");
      Serial.print(String(" : SprayDur = ") + SprayOutput); // spray duration = 0 bc off
    }
  }
  Serial.println("s");
  logFile.println();        //end with a ;
  logFile.close();
}

void readFile(fs::FS& fs, const char* path) {
  //Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  String line = file.readStringUntil('\n'); //clears first line
  file.seek(currentFilePosition);
  
  lineDone = false;
  int i = 0;
  char value[9];
  int valueIndex = 0;
  memset(value, '\0', sizeof(value));  //clear out array

  if (!file.available()) {
    Serial.println("file unavailable");
    AllStop_FXN();
    EndOfFile = true;
    return;
  }
  while (file.available() && !lineDone) {
    char newData = file.read();
    if (newData == ';') {  //done reading in value data point
      data[i] = atof(value);
      i++;
      memset(value, '\0', sizeof(value));  //clear out array
      valueIndex = 0;

    } else if (newData == '\n') {          //done with line of file
      if (MadeitFlag || mode == mode_RC) {
        i = 0;
        lineDone = true;
        currentFilePosition = file.position();
      }
    } else {  //add new character to value array
      if (valueIndex < sizeof(value) - 1) {
        value[valueIndex] = newData;
        valueIndex++;
      }
    }
  }
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
    mode = mode_PAUSE;
    Pause(); 
    Serial.println("DANGER CLOSE");
  } 

  else if ((Mode_dur < 1800) && (Mode_dur > 1200))  //mode RC is turned on
  {
#ifndef RC
    mode = mode_RC;
    mode_Flag = 0;
#endif

    //mode_Flag = 1;
  } else if (Mode_dur > 1800)  // mode Record is turned on
  {
#ifndef RC
    mode = mode_RECORD;
#endif

  } else if ((Mode_dur < 1200) && (mode != mode_All_STOP))  // mode playback is turned on and not coming from all stop
  {
#ifndef RC
    mode = mode_PLAYBACK;
    mode_Flag = 0;

#endif

  } else if ((ResumeDur < 1500) && (DangerFlag == true))  // mode resume is turned on, simply exits pause function
  {
#ifndef RC
    mode = mode_PLAYBACK; //implement resuming back to file position
    DangerFlag = false;
#endif

  } else {
#ifndef RC
    mode = mode_All_STOP;
#endif
  }  // put in 1-19-24 to get rid of residual turning after all stop
}

void OLED_display() {
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
      //display.print(sqrt(errDev1) + sqrt(errDev2));
      break;
    case mode_PLAYBACK:
      display.print("PLAYBACK");
      break;
    case mode_PAUSE:
      display.print("PAUSE");
      break;
    case mode_All_STOP:
      display.print("ALL STOP");
      break;
  }
  display.display();  // Show initial text
}

void Output() {
  rightSideServo.write(RHMIX);  //moves right motor a certain amount
  leftSideServo.write(LHMIX);   //moves left motor a certain amount

  if (SprayOutput) {  //This is where the spray relay gets turned on or off, multiple modes affect this
    ourGPIO.digitalWrite(RelayOutPin, LOW);
  } else {
    ourGPIO.digitalWrite(RelayOutPin, HIGH);
  }
}

void AllStop_FXN() {
  mode = mode_All_STOP;
  RHMIX = Mid_Thrust;         // motors stop
  LHMIX = Mid_Thrust;          // motors stop
  ourGPIO.digitalWrite(RelayOutPin, HIGH);  // spray off
  Lights(0,0,0,0,1,0);
}

void Pause(){
  Lights(0,0,0,1,0,1);
  RHMIX = Mid_Thrust;         // motors stop
  LHMIX = Mid_Thrust;          // motors stop
  ourGPIO.digitalWrite(RelayOutPin, HIGH);  // spray off

  //two things can happen
  //1)  need to move object
  //      hit resume and continue playback
  //2)  need to move robot
  //      switch to rc and move robot
  //      then hit resume and continue playback
}
void Resume(){
  //continue playback from start of line, must recalculate position (especially if robot had to be moved)
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
    pulseInTimeBegin_SpraySens = micros();
  } else {
    Spray_Sensitivity_Dur = micros() - pulseInTimeBegin_SpraySens;
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
