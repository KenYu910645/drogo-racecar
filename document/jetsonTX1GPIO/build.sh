#!/bin/bash
g++ -O2 -Wall exampleGPIOApp.cpp jetsonGPIO.c -o exampleGPIOApp 

g++ -O2 -Wall relayGPIO.cpp jetsonGPIO.c -o relayGPIO


g++ -O2 -Wall relayClose.cpp jetsonGPIO.c -o relayClose
g++ -O2 -Wall relayOpen.cpp  jetsonGPIO.c -o relayOpen
