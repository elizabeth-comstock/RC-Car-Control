/******************************************************
 Controls steering and throttle servo motors using
 joystick input and ROS. 
 
 Version 3 attempts to add a PID autotuner. 
  
 For the full tutorial write up, visit
 www.ros.org/wiki/rosserial_arduino_demos
 
 For more information on the Arduino Servo Library
 Checkout :
 http://www.arduino.cc/en/Reference/Servo
 *****************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt32.h>

///////////////// PID autotuner #includes /////////////////
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
////////////// end of PID autotuner #includes //////////////

ros::NodeHandle n;

Servo steeringServo;
Servo throttleServo;

int steeringValue = 90;
int throttleValue = 100;

// time in seconds since last joy message received before automatic stop
const double noJoyMessageThreshold = 2;
double timeSinceLastJoyMessage = noJoyMessageThreshold;
const unsigned long runFrequency = 60;

bool reverseEnabled = 0;

////////////// PID autotuner working variables //////////////
byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

boolean useSimulation = false; // set to false to connect to the real world
////////// end of PID autotuner working variables //////////

void disconnectCallback(){
  steeringServo.write(90);
  throttleServo.write(100);
}  

void servoCallback(const std_msgs::UInt32& msg){
  throttleValue = msg.data % 1000;
  steeringValue = (msg.data - throttleValue) / 1000;
  
  if (reverseEnabled == 0 && throttleValue >= 103){
    throttleServo.write(127);
    delay(500);
    throttleServo.write(100);
    delay(500);
      
    reverseEnabled = 1;
  }
  
  if (throttleValue <= 95){
    reverseEnabled = 0;
  }
  
  // sets minimum values for throttle to overcome static friction
  // so that car doesn't vibrate on the spot
  // Github users: this is specific to the chassis, tires, and 
  // servo that you're using so please experiment on your own
  if (throttleValue >= 103){
    throttleValue = 111;
  }
  
  else if (throttleValue >= 93 && throttleValue <= 97){
    throttleValue = 93;
  }
  
  throttleServo.write(throttleValue);
  
  timeSinceLastJoyMessage = 0;
}

ros::Subscriber<std_msgs::UInt32> sub("cinnabar", servoCallback);

//////////////////// AUTOTUNE FUNCTIONS ////////////////////

void AutoTuneHelper(boolean start){
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void changeAutoTune(){
  // set the output to the desired starting frequency.
  output=aTuneStartValue;
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  AutoTuneHelper(true);
  tuning = true;
}

void SerialSend(){
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
  Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
  Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
}

void SerialReceive(){
  if(Serial.available()){
    char b = Serial.read(); 
    Serial.flush(); 
    if((b=='1' && !tuning) || (b!='1' && tuning))
      changeAutoTune();
  }
}

void DoModel(){
  // cycle the dead time
  for(byte i=0;i<49;i++){
    theta[i] = theta[i+1];
  }
  // compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;
}

///////////////// END OF AUTOTUNE FUNCTIONS /////////////////

void setup(){
  // opens serial port, sets data rate to 9600 Hz
  Serial.begin(9600);

  n.initNode();
  n.subscribe(sub);

  steeringServo.attach(6); 
  throttleServo.attach(10);
  
  /// this was found under setup() in the PID autotuner code ///
  
  // setup the pid 
  myPID.SetMode(AUTOMATIC);

  serialTime = 0;
  /////////////////// so I'm putting it here ///////////////////
}

void loop(){
  timeSinceLastJoyMessage = timeSinceLastJoyMessage + (1 / runFrequency);

  if (timeSinceLastJoyMessage > noJoyMessageThreshold) 
    disconnectCallback();
  
  n.spinOnce();

  /// this was found under loop() in the PID autotuner code ///
  
  unsigned long now = millis();

  // pull the input in from the real world
  input = steeringValue/*analogRead(0)*/;
  
  myPID.Compute();
  
  steeringServo.write(steeringValue);
  
  // send-receive with processing if it's time
  if(millis()>serialTime){
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
  /////////////////// so I'm putting it here ///////////////////
 
  delay(1000 / runFrequency);
}



