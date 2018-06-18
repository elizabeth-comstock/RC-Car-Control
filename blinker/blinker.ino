/***********************************************************
 This program sets up a ROS node on Arduino, to control turn 
 indicator lights on an autonomous vehicle through a relay. 
 
 The Arduino ROS node subscribes to the topic blinker, 
 which publishes a UInt8 that denotes which blinker light
 should be flashing. The integers are assigned as follows:
 
 1 - turn left
 2 - turn right
 3 - both lights flashing (e.g. when stopping the vehicle
     in the middle of the road)
 
 All others - no lights on
 
 Arduino pin layout is as follows: 
 
 Pin 8 - left blinker
 Pin 9 - right blinker
 ***********************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle n;
const int leftPin = 8;
const int rightPin = 9;
bool doesLeftPinBlink = 0;
bool doesRightPinBlink = 0;

void blinkerCallback(const std_msgs::UInt8& msg){
  switch (msg.data){
    case 1 :
    doesLeftPinBlink = 1;
    doesRightPinBlink = 0;
    break;
    case 2 :
    doesLeftPinBlink = 0;
    doesRightPinBlink = 1;
    break;
    case 3 :
    doesLeftPinBlink = 1;
    doesRightPinBlink = 1;
    break;
    default :
    doesLeftPinBlink = 0;
    doesRightPinBlink = 0;
    break;
  }
}

ros::Subscriber<std_msgs::UInt8> blinkerSub("blinker", blinkerCallback);

void setup(){
  // opens serial port, sets data rate to 9600 Hz
  Serial.begin(9600);

  n.initNode();
  n.subscribe(blinkerSub);

  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
}

void loop(){
  n.spinOnce();
  
  if (doesLeftPinBlink == 1) 
    digitalWrite(leftPin, HIGH);
  else 
    digitalWrite(leftPin, LOW);
    
  if (doesRightPinBlink == 1)
    digitalWrite(rightPin, HIGH);
  else
    digitalWrite(rightPin, LOW);
  
  delay(500);

  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);
  
  delay(500);
}
