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
    jetsonTX1GPIONumber relay = gpio219;
    // Make the button and led available in user space
    cout << "Closing GPIO pin : " << relay << endl;
    gpioExport(relay) ;
    gpioSetDirection(relay,outputPin) ;
    gpioSetValue(relay, 0);
    cout << "GPIO " << relay << " has turn off." << endl;
    //gpioUnexport(relay);     // unexport the LED
    return 0;
}
