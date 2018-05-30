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

double timeSinceLastJoyMessage = 0;

void disconnectCallback(){
  steeringServo.write(90);
  throttleServo.write(100);
}  

void servoCallback(const std_msgs::UInt32& msg){
  int throttleValue = msg.data % 1000;
  int steeringValue = (msg.data - throttleValue) / 1000;
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
  timeSinceLastJoyMessage = timeSinceLastJoyMessage + 0.1;

  if (timeSinceLastJoyMessage > 1) 
    disconnectCallback();
  
  n.spinOnce();
  delay(100);
}
