/*
  Controls the steering and throttle on an RC car. 
  Use with Arduino Uno. 
  
  Version 5 attempts to use the Linux Terminal for keyboard 
  input instead of the Arduino serial monitor. 

  This is how the ESC works:
  When in forward drive mode, press brake once to brake, and 
  then press brake again to reverse. Brake has only one speed. 
  When in reverse mode, press forward to brake and then 
  immediately boost forward. 
  
  From experience, the throttle neutral point PWM signal is an
  uptime of around 1.56 to 1.60 ms. The ESC (electronic speed
  controller) must detect such a signal upon startup before it 
  will run the motor, probably as a safety feature. 
*/

#include <Servo.h> 

// set the pins used to simulate the PWM output to control steering
const int steering = 5;
const int throttle = 6;
// initializes the servo steering motor
Servo steeringServo;
// pretends the ESC is also a servo
Servo throttleServo;
// these values will be passed to the throttle and steering functions at startup. 
// sets the steering to straight and throttle to neutral
int steeringValue = 90;
int throttleValue = 100;
// for incoming serial data
int arrayCount = 0;
char incomingByte[7] = {};
int steeringSerialInterpreterValue = 90;
int throttleSerialInterpreterValue = 100;
  
void setup() {
  // opens serial port, sets data rate to 9600 Hz
  Serial.begin(9600);
  // tells the Arduino that the steering output pin controls a servo
  steeringServo.attach(steering, 1220, 1700);
  // pretends the throttle is a servo
  throttleServo.attach(throttle, 1040, 2000);
}

// PWM output to the steering servo motor
int operateSteering(int steeringValue) {
  steeringServo.write(steeringValue);
}

// PWM output to the throttle servo motor
int operateThrottle(int throttleValue) {
  throttleServo.write(throttleValue);
}

void loop() {
  // the Uno needs to keep sending PWMs to the throttle and steering, 
  // even when no data is being sent from the USB serial port:
  operateSteering(steeringValue);
  operateThrottle(throttleValue);
  
  // writes to array incomingByte if keyboard input detected
  if (Serial.available() > 0) {
    incomingByte[arrayCount] = Serial.read();
    Serial.print(incomingByte[arrayCount]);
    ++arrayCount;
  }
    
  if ( arrayCount >= 7 ) {
    // first 3 digits are for steering
    steeringSerialInterpreterValue = ( (incomingByte[0] - 48) * 100 + (incomingByte[1] - 48) * 10 + (incomingByte[2] - 48) );
    steeringValue = steeringSerialInterpreterValue;

    // last 3 digits are for throttle
    throttleSerialInterpreterValue = ( (incomingByte[3] - 48) * 100 + (incomingByte[4] - 48) * 10 + (incomingByte[5] - 48) );
    throttleValue = throttleSerialInterpreterValue;
    
    // asks Arduino to write to array incomingByte from 0 again
    arrayCount = 0;
  }
}
