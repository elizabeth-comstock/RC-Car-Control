/*
  Controls the steering and throttle on an RC car. On button press, 
  a PWM signal is generated that controls the servo motor, which 
  controls the steering. Use with Arduino Uno. 
  
  Note to self: analogWrite() doesn't work. 
  
  In contrast with steeringAndThrottlePWMv1, this program uses
  the servo libraries (<Servo.h>) and functions (Servo.attach,
  Servo.write) for more responsiveness. You may damage the motors, 
  though.
  
  Old code is commented out. 
  
  From experience, the throttle neutral point PWM signal is an
  uptime of around 1.56 to 1.60 ms. The ESC (electronic speed
  controller) must detect such a signal upon startup before it 
  will run the motor, probably as a safety feature. 
*/

#include <Servo.h> 

// set the pin used to simulate the PWM output to control steering
int steering = 5;
int throttle = 6;
// set the pin for button input
int leftButton = 11;
int rightButton = 12;
int throttleButton = 8; 
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
  pinMode(throttleButton, INPUT_PULLUP);
}

void turnStraight() {
  steeringServo.write(90);
/*  
  // simulated PWM output for going straight
  // the PWM output for going straight is 
  // 1.45 ms high, 14.56 ms low. 
  digitalWrite(steering, HIGH);   
  delayMicroseconds(1450);   
  digitalWrite(steering, LOW);    
  delayMicroseconds(14560);
*/
}

void turnLeft() {
  steeringServo.write(120);
/*
  // simulated PWM output for turning full left
  // the PWM output for turning full left is 
  // 1.70 ms high, 14.34 ms low.
  digitalWrite(steering, HIGH);   
  delayMicroseconds(1700);   
  digitalWrite(steering, LOW);    
  delayMicroseconds(14340);
*/
}

void turnRight() {
  steeringServo.write(60);
/*
  // simulated PWM output for turning full right
  // the PWM output for turning full right is 
  // 1.22 ms high, 14.80 ms low. 
  digitalWrite(steering, HIGH);   
  delayMicroseconds(1220);   
  digitalWrite(steering, LOW);    
  delayMicroseconds(14800);
*/
}

void accelerate() {
  throttleServo.write(80);
/*
  // simulated PWM output for light acceleration
  digitalWrite(throttle, HIGH);   
  delayMicroseconds(1400);   
  digitalWrite(throttle, LOW);    
  delayMicroseconds(14640);
*/
}

void brake() {
  throttleServo.write(100);
/*  
  // simulated PWM output for braking
  digitalWrite(throttle, HIGH);   
  delayMicroseconds(1560);   
  digitalWrite(throttle, LOW);    
  delayMicroseconds(14480);
*/
}

// the loop routine runs over and over again forever:
void loop() {
  //read the pushbutton value into a variable
  int sensorValLeft = digitalRead(leftButton);
  int sensorValRight = digitalRead(rightButton);
  int sensorValThrottle = digitalRead(throttleButton);
  
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
  
  if (sensorValThrottle == LOW) {
    accelerate();
  }
  
  else {
    brake();
  }
}
