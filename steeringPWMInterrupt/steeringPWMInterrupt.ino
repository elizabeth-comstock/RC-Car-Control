/*
  Controls the steering on an RC car. On button press, a PWM signal
  is generated that controls the servo motor, which controls the
  steering. Use with Arduino Uno. 
  
  I am a new programmer and use GitHub as a personal repository. 
  
  This code is for a very specific model of RC car. Do not attempt
  to use it; the PWM signals required for your model are different,
  and the code will likely cause your machine to run out of control.
  
  Code adapted from LED and DigitalInputPullUp examples in the 
  Arduino IDE. 
  
  Interrupts added! 
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.allaboutcircuits.com/technical-articles/using-interrupts-on-arduino/
 */

// Definition of interrupt names
// #include < avr/io.h >
// ISR interrupt service routine
// #include < avr/interrupt.h >

// set the pin used to simulate the PWM output to control steering
// use a PWM-enabled pin, but avoid pins 5 and 6 since they have a higher clock rate
const int steeringOutput = 13;
const int leftButton = 2;
const int rightButton = 4;

volatile int sensorValLeft = digitalRead(2);
volatile int sensorValRight = digitalRead(4);
// Keep in mind the pullup means the pushbutton's
// logic is inverted. It goes HIGH when it's open,
// and LOW when it's pressed.

// setup code runs once
void setup() {                
  // initialize the steering pin as an output.
  pinMode(steeringOutput, OUTPUT); 
  // initialize the button pins as inputs. 
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  // attach interrupts to the ISR vector.
  attachInterrupt(digitalPinToInterrupt(leftButton), steer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightButton), steer, CHANGE);
}

// simulated PWM output for going straight
// the PWM output for going straight is 1.45 ms high, 14.56 ms low. 
void turnStraight() {
  for(int x = 0; x == 4; x++) {
    digitalWrite(steeringOutput, HIGH);   
    delayMicroseconds(1450);   
    digitalWrite(steeringOutput, LOW);    
    delayMicroseconds(14560);
  }
}

// simulated PWM output for turning full left
// the PWM output for turning full left is 1.70 ms high, 14.34 ms low. 
void turnLeft() {
  for(int x = 0; x == 4; x++) {
    digitalWrite(steeringOutput, HIGH);   
    delayMicroseconds(1700);   
    digitalWrite(steeringOutput, LOW);    
    delayMicroseconds(14340);
  }
}

// simulated PWM output for turning full right
// the PWM output for turning full right is 1.22 ms high, 14.80 ms low. 
void turnRight() {
  for(int x = 0; x == 4; x++) {
    digitalWrite(steeringOutput, HIGH);   
    delayMicroseconds(1220);   
    digitalWrite(steeringOutput, LOW);    
    delayMicroseconds(14800);
  }
}

// loop runs over and over again
void loop() {
  // nothing here! only executes when there's an interrupt
}

void steer() {
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
} 
