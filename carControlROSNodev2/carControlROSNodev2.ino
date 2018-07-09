/******************************************************
 Controls steering and throttle servo motors using
 joystick input and ROS. 
 Version 2 attempts to add a "stop code", so that the
 car won't continue running if the Arduino, Jetson, or
 joystick suddenly disconnect or encounter an error.
  
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

void disconnectCallback(){
  steeringServo.write(90);
  throttleServo.write(100);
}  

void servoCallback(const std_msgs::UInt32& msg){
  throttleValue = msg.data % 1000;
  steeringValue = (msg.data - throttleValue) / 1000;
  
  if (reverseEnabled == 0 && throttleValue >= 110){
    throttleServo.write(127);
    delay(500);
    throttleServo.write(100);
    delay(500);
      
    reverseEnabled = 1;
  }
  
  if (throttleValue <= 95){
    reverseEnabled = 0;
  }
  
  if (throttleValue >= 103){
    throttleValue = 103;
  }
  
  steeringServo.write(steeringValue);
  throttleServo.write(throttleValue);
  
  timeSinceLastJoyMessage = 0;
}

ros::Subscriber<std_msgs::UInt32> sub("cinnabar", servoCallback);

void setup(){
  // opens serial port, sets data rate to 9600 Hz
  Serial.begin(9600);

  n.initNode();
  n.subscribe(sub);

  steeringServo.attach(9); 
  throttleServo.attach(10);
}

void loop(){
  timeSinceLastJoyMessage = timeSinceLastJoyMessage + (1 / runFrequency);

  if (timeSinceLastJoyMessage > noJoyMessageThreshold) 
    disconnectCallback();
  
  n.spinOnce();
 
  delay(1000 / runFrequency);
}
