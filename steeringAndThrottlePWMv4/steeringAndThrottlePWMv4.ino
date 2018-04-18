/*
  Controls the steering and throttle on an RC car. 
  Use with Arduino Uno. 
  
  Version 4 changes the input from GPIO to USB port, and adds
  fine control of throttle and steering. 

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
// only for incoming serial data
int incomingByte = 0;
int serialInputValue = 0;
  
void setup() {
  // opens serial port, sets data rate to 9600 Hz
  Serial.begin(9600);
  // tells the Arduino that the steering output pin controls a servo
  steeringServo.attach(steering, 1220, 1700);
  // pretends the throttle is a servo
  throttleServo.attach(throttle, 1040, 2000);
  // initialisation message
  Serial.print("Enter an integer to control the car! ENSURE NEWLINE IS TURNED ON.\n");
  Serial.print("   120 -- full left,      90 -- straight,   60 -- full right\n");
  Serial.print("  1050 -- full throttle, 1100 -- neutral, 1148 -- full reverse\n");
  Serial.print("          Simulate double-tap brake for full reverse!");
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
  
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // throw away previous serialInputValue
    serialInputValue = 0;         

    // force into a loop until '\n' is received
    while(1) {            
      incomingByte = Serial.read();
      // exit the while(1), we're done receiving
      if (incomingByte == '\n') break;   
      // if no characters are in the buffer read() returns -1
      if (incomingByte == -1) continue;  
      // shift left 1 decimal place
      serialInputValue *= 10;  
      // convert ASCII to integer, add, and shift left 1 decimal place
      serialInputValue = ((incomingByte - 48) + serialInputValue);
    }

    // debug, print what was received:
    Serial.print("I received: ");
    Serial.println(serialInputValue); 

    if (serialInputValue >= 60 && serialInputValue <= 120) {
      steeringValue = serialInputValue;
    }

    else if (serialInputValue >= 1050 && serialInputValue <= 1148) {
      throttleValue = serialInputValue - 1000;
    }
    
    else {
      throttleValue = 100;
    }
  }
}
