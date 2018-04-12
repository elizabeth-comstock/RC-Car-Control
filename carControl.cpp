/*                          |      | |
 *
 *                          | |    | _
 */

/* If this code doesn't work for the TX2, you might want to try:
 *   - Changing the GPIO pin numbers here and in the header file. 
 *   - Enabling GPIO for application space: https://elinux.org/GPIO
 *     Type into terminal:
 *
 *     sudo -i                       (goes into root user)
 *     GPIO=398
 *     cd /sys/class/gpio
 *     ls
 *     echo $GPIO > export
 *     ls
 *
 *     Repeat for GPIO=481, or whatever GPIO pin you're using.
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "jetsonGPIO.h"
using namespace std;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main(int argc, char *argv[]){

    cout << "Testing the GPIO Pins" << endl;

    // Defines an enum for throttle statuses
    enum ThrottleStatus {
	tReverse = 0, 
	tNeutral,
	tForward
    } ;

    // The names tell you how to connect the GPIO pins from the TX2 to the Arduino
    // toThrottleButton, for example, corresponds with throttleButton on the Arduino code
    TX2_GPIO toLeftButton    = gpio301 ;
    TX2_GPIO toRightButton   = gpio302 ;
    TX2_GPIO toForwardButton = gpio300 ;
    TX2_GPIO toReverseButton = gpio299 ;
    
    // Make the button and led available in user space
    gpio_export(toLeftButton) ;
    gpio_export(toRightButton) ;
    gpio_export(toForwardButton) ;
    gpio_export(toReverseButton) ;

    gpio_set_direction(toLeftButton,GPIO_DIRECTION_OUTPUT) ;
    gpio_set_direction(toRightButton,GPIO_DIRECTION_OUTPUT) ;
    gpio_set_direction(toForwardButton,GPIO_DIRECTION_OUTPUT) ;
    gpio_set_direction(toReverseButton,GPIO_DIRECTION_OUTPUT) ;

    // Wait for the push button to be pressed
    cout << "Please mash WASD! ESC key quits the program" << endl ;

    gpio_set_value(toLeftButton,GPIO_PIN_VALUE_HIGH) ;
    gpio_set_value(toRightButton,GPIO_PIN_VALUE_HIGH) ;
    gpio_set_value(toForwardButton,GPIO_PIN_VALUE_HIGH) ;
    gpio_set_value(toReverseButton,GPIO_PIN_VALUE_HIGH) ;

    char keyValue = 'p';
    // Creates an enum to record the throttle status, and sets it to neutral
    ThrottleStatus throttleStatus = tNeutral ;

    for( ; keyValue != 27 ; ) {
	
	keyValue = getkey();

	// Press a to turn left
	if(keyValue == 'a') {
	    gpio_set_value(toLeftButton,GPIO_PIN_VALUE_LOW) ;
	    gpio_set_value(toRightButton,GPIO_PIN_VALUE_HIGH) ;
	}

	// Press d to turn right
	else if(keyValue == 'd') {
	    gpio_set_value(toLeftButton,GPIO_PIN_VALUE_HIGH) ;
	    gpio_set_value(toRightButton,GPIO_PIN_VALUE_LOW) ;
	}

	// Press f to pay respects
	else if(keyValue == 'f') {
	    gpio_set_value(toLeftButton,GPIO_PIN_VALUE_HIGH) ;
	    gpio_set_value(toRightButton,GPIO_PIN_VALUE_HIGH) ;
	}

	// Press s to stop
	if(keyValue == 's') {
	    if(throttleStatus == tForward) {
		throttleStatus = tNeutral ;
		gpio_set_value(toForwardButton,GPIO_PIN_VALUE_HIGH) ;
		gpio_set_value(toReverseButton,GPIO_PIN_VALUE_LOW) ;
	    }
	    else {
		gpio_set_value(toReverseButton,GPIO_PIN_VALUE_HIGH) ;

		usleep(100000) ;

		throttleStatus = tReverse;
		gpio_set_value(toForwardButton,GPIO_PIN_VALUE_HIGH) ;
		gpio_set_value(toReverseButton,GPIO_PIN_VALUE_LOW) ;
	    }
	}

	// Press w to go forward
	else if(keyValue == 'w') {
	    if(throttleStatus == tReverse) {
		throttleStatus = tNeutral ;
		gpio_set_value(toForwardButton,GPIO_PIN_VALUE_HIGH) ;
		gpio_set_value(toReverseButton,GPIO_PIN_VALUE_HIGH) ;
	    }
	    else {
		throttleStatus = tForward;
		gpio_set_value(toForwardButton,GPIO_PIN_VALUE_LOW) ;
		gpio_set_value(toReverseButton,GPIO_PIN_VALUE_HIGH) ;
	    }
	}

	usleep(1000);
    }

    cout << "\nPhosphophyllite lost her head." << endl;
    gpio_unexport(toLeftButton) ;
    gpio_unexport(toRightButton) ;
    gpio_unexport(toForwardButton) ;
    gpio_unexport(toReverseButton) ;
    return 0;
}


