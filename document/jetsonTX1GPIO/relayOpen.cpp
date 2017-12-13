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
    cout << "Opening GPIO pin : " << relay << endl;
    gpioExport(relay) ;
    gpioSetDirection(relay,outputPin) ;
    gpioSetValue(relay, 1);
    cout << "GPIO " << gpio219 << " has turn on." << endl;
    //gpioUnexport(relay);     // unexport the LED
    return 0;
}
