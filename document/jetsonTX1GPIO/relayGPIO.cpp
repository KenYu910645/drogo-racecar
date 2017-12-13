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

int main(int argc, char *argv[])
{
    cout << "Testing the GPIO Pins" << endl;
    
    jetsonTX1GPIONumber relay = gpio219;
    // Make the button and led available in user space
    cout << "using GPIO pin : " << relay << endl;
    gpioExport(relay) ;
    gpioSetDirection(relay,outputPin) ;
    // Reverse the button wiring; this is for when the button is wired
    // with a pull up resistor
    // gpioActiveLow(relay, true);
    for (int i = 0;i < 5 ;i++)
    {
        cout << "Setting the LED on" << endl;
        int state = gpioSetValue(relay, 1);
	if (state != 0) cerr << "can't set on" << endl;
        usleep(1000000);         // on for 3s
        cout << "Setting the LED off" << endl;
        state = gpioSetValue(relay, 0);
	if (state != 0) cerr << "can't set off" << endl;
        usleep(1000000);         // on for 3s
    }
    
    cout << "GPIO example finished." << endl;
    gpioUnexport(relay);     // unexport the LED
    return 0;
}
