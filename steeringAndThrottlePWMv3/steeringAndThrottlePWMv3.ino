/*
  Controls the steering and throttle on an RC car. 
  Use with Arduino Uno. 
  
  Version 3 adds a reverse function that was found to be
  supported by hardware, i.e. the RC car. Woe is me.

  This is how the ESC works:
  When in forward drive mode, press brake once to brake, and 
  then press brake again to reverse. Brake has only one speed. 
  When in reverse mode, press forward to brake and then 
  immediately boost forward. 

  The input pins on the Uno need to continuously receive a signal
  in order for the output pins to run the drive motor. 
  
  From experience, the throttle neutral point PWM signal is an
  uptime of around 1.56 to 1.60 ms. The ESC (electronic speed
  controller) must detect such a signal upon startup before it 
  will run the motor, probably as a safety feature. 
*/

#include <Servo.h> 

// set the pins used to simulate the PWM output to control steering
int steering = 5;
int throttle = 6;
// set the pins for button input
int leftButton = 11;
int rightButton = 12;
int forwardButton = 8;
int reverseButton = 9; 
// initializes the servo steering motor
Servo steeringServo;
// pretends the ESC is also a servo
Servo throttleServo;

void setup() {
  // tells the Arduino that the steering output pin controls a servo
  steeringServo.attach(steering, 1220, 1700); // was pinMode(steering, OUTPUT); 
  // pretends the throttle is a servo
  throttleServo.attach(throttle, 1040, 2000); // was pinMode(throttle, OUTPUT);
  // initialize the button pins as inputs. 
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(forwardButton, INPUT_PULLUP);
  pinMode(reverseButton, INPUT_PULLUP);
}

void turnStraight() {
  steeringServo.write(90);
}

void turnLeft() {
  steeringServo.write(120);
}

void turnRight() {
  steeringServo.write(60);
}

void accelerate() {
  throttleServo.write(90);
}

void brake() {
  throttleServo.write(100);
}

void reverse() {
  throttleServo.write(148);
}

void loop() {
  //read the pushbutton value into a variable
  int sensorValLeft = digitalRead(leftButton);
  int sensorValRight = digitalRead(rightButton);
  int sensorValForward = digitalRead(forwardButton);
  int sensorValReverse = digitalRead(reverseButton);
  
  // Keep in mind the pullup means the pushbutton's
  // logic is inverted. It goes HIGH when it's open,
  // and LOW when it's pressed.
  
  // if no input is detected, go straight:
  if (sensorValLeft == HIGH && sensorValRight == HIGH) {
    turnStraight();
  }
  
  else if (sensorValLeft == LOW && sensorValRight == LOW) {
    turnStraight();
  }  
  
  else if (sensorValLeft == LOW && sensorValRight == HIGH) {
    turnLeft();
  }
  
  else if (sensorValLeft == HIGH && sensorValRight == LOW) {
    turnRight();
  }
  
  if (sensorValForward == LOW) {
    accelerate();
  }

  else if (sensorValReverse == LOW) {
    reverse();
  }
  
  else {
    brake();
  }
}
