/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle n;

Servo steeringServo;

void steeringServoCallback(const std_msgs::UInt8& msg){
  steeringServo.write(msg.data);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}

ros::Subscriber<std_msgs::UInt8> steering_sub("steering", steeringServoCallback);

void setup(){
  // opens serial port, sets data rate to 9600 Hz
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  n.initNode();
  n.subscribe(steering_sub);

  steeringServo.attach(9); //attach it to pin 9
}

void loop(){
  n.spinOnce();
  delay(10);
}

