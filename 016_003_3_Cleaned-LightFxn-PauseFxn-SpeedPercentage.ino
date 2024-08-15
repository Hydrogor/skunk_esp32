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
// 8-13-24 016-003_2 orientation error works 
// 8-14-24 016-003_3 Code cleaned up, added lights fxn and pause fxn and scaled speed
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

#define MaxSpeed 65           // Max Speed in Percentage 

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

#define mode_RECORD 0    //enumeration of modes
#define mode_RC 1        //enumeration of modes
#define mode_PLAYBACK 2  //enumeration of modes
#define mode_PAUSE 3     //enumeration of modes
#define mode_All_STOP 4  //enumeration of modes

#define DANGER_DIST 0              //ultrasonic distance for danger stop
#define CONCERN_DIST 500           //ultrasonic distance for concern
#define SLAVE_BROADCAST_ADDR 0x00  //default address for ultrasonic
#define SLAVE_ADDR 0x00            //SLAVE_ADDR 0xA0-0xAF for ultrasonic
#define NUMBER_OF_SENSORS 2

#define SteeringPin 27
#define ThrustPin 35
#define SpraySensPin 34     //ARTEMIS PIN CHANGE per 012-002
#define SteeringSensPin 32  //ARTEMIS PIN CHANGE per 012-002
#define RelayPin 25         //ARTEMIS PIN CHANGE per 012-002
#define ModePin 33          //ARTEMIS PIN CHANGE per 012-002
#define rightSidePin 16     //out to motor controller
#define leftSidePin 17      //out to motor controller

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

MovingAverage<long, 16> LHFilter;   //ultrasonic
MovingAverage<long, 16> RHFilter;   //ultrasonic
MovingAverage<long, 16> CEFilter;   //ultrasonic

uint8_t distance_H = 0;  //ultrasonic
uint8_t distance_L = 0;  //ultrasonic
uint16_t distance = 0;   //ultrasonic

volatile unsigned long pulseInTimeBegin_Steering = micros();      //Interrupt timers
volatile unsigned long pulseInTimeBegin_Thrust = micros();        //Interrupt timers
volatile unsigned long pulseInTimeBegin_Relay = micros();         //Interrupt timers
volatile unsigned long pulseInTimeBegin_Mode = micros();          //Interrupt timers
volatile unsigned long pulseInTimeBegin_ModeSens = micros();      //Interrupt timers
volatile unsigned long pulseInTimeBegin_SteeringSens = micros();  //Interrupt timers

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars], messageFromPC[numChars] = { 0 }, logMessage[numChars];
int integerFromPC = 0, anchor1 = 1780, anchor2 = 1781;
float floatFromPC = 0.0, anchorDistance1, anchorDistance2;
bool newData = false, mode_Flag = false;
float LHMIX, RHMIX;
float Sc_MaxSteer, Sc_MinSteer, Sc_MaxThrust, Sc_MinThrust, SpeedCap; //scaled max n mins
int mode = 1;  //mode is record or playback
float Direction, Thrust, ThetaPath;
volatile long SteeringDur = 1500, ThrustDur = 1500;
long RelayDur = 2000;
long SprayDuration;
long Spray_Sensitivity_Dur = 1500;
volatile long Steering_Sensitivity_Dur = 1500;
long Mode_dur = 1000;
int startTime;
int startTime_Playback;
long currentFilePosition = 0;
long RTime, dt;
int ultraSonic[] = { 0x22, 0x23 };  //used to check if ultra was connected or not
bool RelayFlag = false;
bool isPaused = false;
bool EndOfFile = false;
bool SprayOutput = false;
bool recordFirstTime = false;
float data[7];  //fitness is the root mean squared of how well the robot followed the path.
float errorTotal1 = 0, errorTotal2 = 0;
float errorAnchor1 = 0, errorAnchor2 = 0;
float K_total, K_heading, K_location = 0.1;
int LEDTimer = 1000; // 1sec pulse
EasyBNO055_ESP bno;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  bno.start();  //starts up BNO IMU // without one hooked up this freezes the code
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1, 13, 14);  // Rx = 13, Tx = 14 comm. to dw1000 anchor boards
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
  timer = timerBegin(1000000);  // timer 0, prescalar: 80, UP counting
  //timerAttachInterrupt(timer, &onTimer, true);  // Attach interrupt
  timerAttachInterrupt(timer, &onTimer);  // Attach interrupt
  //timerAlarmWrite(timer, 100000, true);         // Match value= 1000000 for 1 sec. delay.
  timerAlarm(timer, 250000, true, 0);  // Match value= 1000000 for 1 sec. delay.
  //timerAlarmEnable(timer);                // Enable Timer with interrupt (Alarm Enable)
  ////////////////////////// INTERIOR HARDWARE TIMER ////////////////////////////////////////////////

  ourGPIO.digitalWrite(WhitePin, LOW);  //all LEDs off
  ourGPIO.digitalWrite(BluePin, LOW);
  ourGPIO.digitalWrite(GreenPin, HIGH);
  ourGPIO.digitalWrite(OrangePin, LOW);
  ourGPIO.digitalWrite(RedPin, LOW);  //all LEDs off
}

//****************************MAIN LOOP**************************************************************************************************
//******************************************************************************************************************************************************
void loop() {
  // Serial.println("..................................LOOP.......................................");
  // Serial.println(String("orientationZ = ") + bno.orientationZ);

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
    double previousError1 = 0, previousError2 = 0;

    //localEnvironmentCheck(); //check local sensors

    startTime = millis();
    dt = millis() - startTime_Playback;

    if ((dt > data[0]) && (mode != mode_All_STOP)) {
      readFile(SD, "/log.txt");                                                  // checks times and read next file line to get new instructions
      LHMIX = data[1];                                                           // setting LH drive to what came in from file
      RHMIX = data[2];                                                           // setting RH drive to what came in from file
      SprayOutput = data[3];                                                     // setting spray trigger to what came in from playback file
      errorAnchor1 = (data[4] - anchorDistance1) * (data[4] - anchorDistance1);  // finding error squared between recorded anchor and current anchor
      errorAnchor2 = (data[5] - anchorDistance2) * (data[5] - anchorDistance2);  // finding error squared between recorded anchor and current anchor
      errorTotal1 = errorTotal1 + errorAnchor1;
      errorTotal2 = errorTotal2 + errorAnchor2;
      ThetaPath = data[6];                                                       // setting the current heading to thetapath
      float CurrentHeading = bno.orientationZ;

      Serial.print(String("time = ") + data[0]);
      Serial.print(String(" : Current_ORIEN = ") + CurrentHeading + String(" : REC_Orien = ") + data[6]);
      // Serial.print(String(" : errAnch1 = ") + errorAnchor1 + String(" : errAnch2 = ") + errorAnchor2);
      // Serial.print(String(" : LHMIX_Rec = ") + data[1] + String(" : RHMIX_Rec = ") + data[2]);

      // Error correction based on ThetaPath
      float headingError = CurrentHeading - ThetaPath;  //with imu
      //Serial.print(String(" : HeadingErrPRE = ") + headingError);
      if (abs(headingError) < 10) K_heading = .09; // conservative correction
      if (abs(headingError) > 10) {                // aggressive correction
        K_heading = .25;
        if ((LHMIX > 105) && (RHMIX > 105)){
          LHMIX -= 10;
          RHMIX -= 10;
        }
      }
      //Serial.print(String(" : HeadingErrPOST = ") + (headingError * K_heading) + String(" : K heading = ") + K_heading);
      // Error correction based on anchor errors



      
      //Serial.print(String(" : AnchorErr = ") + (errorAnchor1 + errorAnchor2) + String(" : K location = ") + K_location);
      // Combine heading error and anchor errors for correction
      float totalError = headingError * K_heading;// + (errorAnchor1 + errorAnchor2) * K_location;  // anchorerrorconstant = 0.01
      Serial.print(String(" : TotalErr = ") + totalError);
      //if (abs(totalError) < 10) K_total = .09; // conservative correction
      //if (abs(totalError) > 10) K_total = .25; // aggressive correction

      // Apply correction to drive towards ThetaPath and reduce anchor errors
      if (headingError < -5){
        LHMIX -= totalError;
      }
      if (headingError > 5){
        RHMIX += totalError;
      }
      if (headingError > -5 && headingError < 5){
        LHMIX -= totalError;// * K_total;
        RHMIX += totalError;// * K_total;
      }

      //Serial.print(String(" : LHMIXwErr = ") + LHMIX + String(" : RHMIXwErr = ") + RHMIX);
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
}

void MIXRC() {  //pulse in for RC PWM and filters it to output LHMIX,RHMIX

  SpeedCap = MaxSpeed / 100; //0-100 to 0-1

  Sc_MaxSteer = ((Max_Steering - Mid_Steering) * SpeedCap) + Mid_Steering; 
  Sc_MinSteer = (Mid_Steering * (1-SpeedCap)) + Mid_Steering; 

  Sc_MaxThrust = ((Max_Thrust - Mid_Thrust) * SpeedCap) + Mid_Thrust; 
  Sc_MinThrust = (Mid_Thrust * (1-SpeedCap)) + Mid_Thrust; 

  Direction = SteeringDur;
  Thrust = ThrustDur;

  // Serial.println(String("Thrust = ") + Thrust + String(" : Direction = ") + Direction);
  Direction = (Direction - LeftDirIn) * (Sc_MaxSteer - Sc_MinSteer);
  Direction = Direction / (RightDirIn - LeftDirIn);
  Direction = Direction + Sc_MinSteer;

  Thrust = (Thrust - BackwardThrustIn) * (Sc_MaxThrust - Sc_MinThrust);
  Thrust = Thrust / (ForwardThrustIn - BackwardThrustIn);
  Thrust = Thrust + Sc_MinThrust;

  if (Direction > Sc_MaxSteer) Direction = Sc_MaxSteer;  //setting caps
  if (Direction < Sc_MinSteer) Direction = Sc_MinSteer;
  if (Thrust > Sc_MaxThrust) Thrust = Sc_MaxThrust;
  if (Thrust < Sc_MinThrust) Thrust = Sc_MinThrust;

  LHMIX = Thrust + (Direction - 90) * (Steering_Sensitivity_Dur * .05) / 100;  //doing a little DJ Dan and the fresh tunes mixing
  RHMIX = Thrust - (Direction - 90) * (Steering_Sensitivity_Dur * .05) / 100;

  // Serial.println(String("LHMIX = ") + LHMIX + String(" : RHMIX = ") + RHMIX);
}

void Spray() {
  if ((SprayDuration < 1500) && (RelayFlag != 1)) {  //this causes problems when there is playback!!!!
    //turn on relay
    RTime = micros();
    RelayFlag = 1;  // flag to say relay is on
    SprayOutput = true;
    ourGPIO.digitalWrite(RelayOutPin, HIGH);
  }

  if ((micros() - RTime) > (Spray_Sensitivity_Dur * 6920 - 5390560)) {
    SprayOutput = false;
    ourGPIO.digitalWrite(RelayOutPin, LOW);  //turn relay off
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
  skunkLog.print(tim);  //data[0]
  skunkLog.print(";");
  skunkLog.print(LHMIX);  //data[1]
  skunkLog.print(";");
  skunkLog.print(RHMIX);  //data[2]
  skunkLog.print(";");
  skunkLog.print(SprayOutput);  //data[3]
  skunkLog.print(";");
  skunkLog.print(anchorDistance1);  //data[4]
  skunkLog.print(";");
  skunkLog.print(anchorDistance2);  //data[5]
  skunkLog.print(";");
  skunkLog.print(bno.orientationZ);  //data[6]

  Serial.println(String("time = ") + tim + String(" : LHMIX = ") + LHMIX + String(" : RHMIX = ") + RHMIX + String(" : orientationZ = ") + bno.orientationZ);

  skunkLog.println(";"); //end with a ;
  skunkLog.close();
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
  if (mode == mode_PAUSE) {Serial.print("PAUSE: SOMETHING IN DA WAY"); }
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
  if (mode == mode_PAUSE) {Serial.print("PAUSE: SOMETHING IN DA WAY"); }
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
    Pause();
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
    case mode_PAUSE:
      display.print("PAUSE");
      break;
    case mode_All_STOP:
      display.print("ALL STOP");
      break;
  }
  display.display();  // Show initial text
  //Serial.println("done");
}

void Output() {
  rightSideServo.write(RHMIX);
  leftSideServo.write(LHMIX);

  if (SprayOutput) {
    ourGPIO.digitalWrite(RelayOutPin, HIGH);
  } else {
    ourGPIO.digitalWrite(RelayOutPin, LOW);
  }
}

void OpenFile(fs::FS& fil) {
  File skunkLog = fil.open("/log.txt", FILE_WRITE);

  if (!skunkLog) {
    Serial.println("Failed to open log.txt file for writing");
  } else {
    Serial.println("open file");
  }  // Determine the total length needed for the concatenated strin
}
////////////////////////////////////////////////////////// READ FILE /////////////////////////////////////////////////////////////////////////////
void readFile(fs::FS& fs, const char* path) {
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
  //Serial.println(file.available());
  if (!file.available()) {
    AllStop_FXN();
    EndOfFile = true;
  }
  while (file.available() && !lineDone) {
    char newData = file.read();
    if (newData == ';') {  //done reading in value data point
      data[i] = atof(value);
      i = i + 1;
      memset(value, '\0', sizeof(value));  //clear out array
    } else if (newData == '\n') {          //done with line of file
      i = 0;
      lineDone = true;
      currentFilePosition = file.position();
    } else {  //add new character to value array
      int length = strlen(value);
      value[length] = newData;
    }
  }
}

void AllStop_FXN() {
  Lights(0,0,0,0,1,0);
  mode = mode_All_STOP;
  rightSideServo.write(90);   // motors stop
  leftSideServo.write(90);    // motors stop
  ourGPIO.digitalWrite(RelayOutPin, LOW);
}

void Pause() {
  if (isPaused) {
    isPaused = false;
    startTime_Playback = millis() - dt;
    Lights(0,0,0,1,0,1);
    mode = mode_PAUSE;
    rightSideServo.write(90);   // motors stop
    leftSideServo.write(90);    // motors stop
  } else {
    isPaused = true;
  }
}

void Lights(bool White, bool Blue, bool Green, bool Orange, bool Red, bool Pulse) { // lights function
  unsigned long currentMillis = millis();
  unsigned long previousMillis = 0;

  // Turn off all LEDs that are not specified
  if (!White) ourGPIO.digitalWrite(WhitePin, LOW);
  if (!Blue) ourGPIO.digitalWrite(BluePin, LOW);
  if (!Green) ourGPIO.digitalWrite(GreenPin, LOW);
  if (!Orange) ourGPIO.digitalWrite(OrangePin, LOW);
  if (!Red) ourGPIO.digitalWrite(RedPin, LOW);

  if (Pulse) {
    if (currentMillis - previousMillis >= LEDTimer) {
      previousMillis = currentMillis;
      
      // Toggle the LEDs based on the input
      if (White && ourGPIO.digitalRead(WhitePin) == LOW) {
        ourGPIO.digitalWrite(WhitePin, HIGH);
      } else if (White) {
        ourGPIO.digitalWrite(WhitePin, LOW);
      }
      if (Blue && ourGPIO.digitalRead(BluePin) == LOW) {
        ourGPIO.digitalWrite(BluePin, HIGH);
      } else if (Blue) {
        ourGPIO.digitalWrite(BluePin, LOW);
      }
      if (Green && ourGPIO.digitalRead(GreenPin) == LOW) {
        ourGPIO.digitalWrite(GreenPin, HIGH);
      } else if (Green) {
        ourGPIO.digitalWrite(GreenPin, LOW);
      }
      if (Orange && ourGPIO.digitalRead(OrangePin) == LOW) {
        ourGPIO.digitalWrite(OrangePin, HIGH);
      } else if (Orange) {
        ourGPIO.digitalWrite(OrangePin, LOW);
      }
      if (Red && ourGPIO.digitalRead(RedPin) == LOW) {
        ourGPIO.digitalWrite(RedPin, HIGH);
      } else if (Red) {
        ourGPIO.digitalWrite(RedPin, LOW);
      }
    }
  } else {
    // Turn on the LEDs without pulsing
    if (White) ourGPIO.digitalWrite(WhitePin, HIGH);
    if (Blue) ourGPIO.digitalWrite(BluePin, HIGH);
    if (Green) ourGPIO.digitalWrite(GreenPin, HIGH);
    if (Orange) ourGPIO.digitalWrite(OrangePin, HIGH);
    if (Red) ourGPIO.digitalWrite(RedPin, HIGH);
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
