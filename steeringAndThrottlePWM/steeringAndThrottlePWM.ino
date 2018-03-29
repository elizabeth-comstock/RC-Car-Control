/*
  Controls the steering and throttle on an RC car. On button press, 
  a PWM signal is generated that controls the servo motor, which 
  controls the steering. Use with Arduino Uno. 
  
  I am a new programmer and use GitHub as a personal repository. 
  
  This code is for a very specific model of RC car. Do not attempt
  to use it; the PWM signals required for your model are different,
  and the code will likely cause your machine to run out of control.
  
  Code adapted from LED and DigitalInputPullUp examples in the 
  Arduino IDE. 
 */

// set the pin used to simulate the PWM output to control steering
// use a PWM-enabled pin, but avoid pins 5 and 6 since they have a higher clock rate
int steering = 13;
int throttle = 11;
int leftButton = 2;
int rightButton = 4;
int throttleButton = 6; 

void setup() {                
  // initialize the steering and throttle pins as outputs.
  pinMode(steering, OUTPUT); 
  pinMode(throttle, OUTPUT);
  // initialize the button pins as inputs. 
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  pinMode(throttleButton, INPUT_PULLUP);
}

// simulated PWM output for going straight
// the PWM output for going straight is 1.45 ms high, 14.56 ms low. 
void turnStraight() {
  digitalWrite(steering, HIGH);   
  delayMicroseconds(1450);   
  digitalWrite(steering, LOW);    
  delayMicroseconds(14560);
}

// simulated PWM output for turning full left
// the PWM output for turning full left is 1.70 ms high, 14.34 ms low. 
void turnLeft() {
  digitalWrite(steering, HIGH);   
  delayMicroseconds(1700);   
  digitalWrite(steering, LOW);    
  delayMicroseconds(14340);
}

// simulated PWM output for turning full right
// the PWM output for turning full right is 1.22 ms high, 14.80 ms low. 
void turnRight() {
  digitalWrite(steering, HIGH);   
  delayMicroseconds(1220);   
  digitalWrite(steering, LOW);    
  delayMicroseconds(14800);
}

// simulated PWM output for accelerating
void accelerate() {
  digitalWrite(throttle, HIGH);   
  delayMicroseconds(1400);   
  digitalWrite(throttle, LOW);    
  delayMicroseconds(14640);
}

// simulated PWM output for braking
void brake() {
  digitalWrite(throttle, HIGH);   
  delayMicroseconds(1560);   
  digitalWrite(throttle, LOW);    
  delayMicroseconds(14480);
}

// the loop routine runs over and over again forever:
void loop() {
  //read the pushbutton value into a variable
  int sensorValLeft = digitalRead(2);
  int sensorValRight = digitalRead(4);
  int sensorValThrottle = digitalRead(6);
  
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
