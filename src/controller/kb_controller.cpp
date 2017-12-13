#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <termios.h>
#include <time.h>
#include "JHPWMPCA9685.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

// Calibrated for a Robot Geek RGS-13 Servo
// Make sure these are appropriate for the servo being used!

// The 666 num mean hell
const int RESET_CODE = 666;
 
const int RC_GO_MAX = 1984; 
const int RC_GO_MIN = 1060;
const int RC_GO_MID = 1500;
const int RC_TURN_MAX = 1984;
const int RC_TURN_MIN = 1032;
const int RC_TURN_MID = 1500;

//limitation
const int TURN_MAX = 100;
const int TURN_MIN = 0;
const int GO_MAX = 60;

// middle point
const int TURN_MID = 50;
const int GO_MID = 50;

//mapping range
const int servoMin = 320 ;
const int servoMax = 520 ;
const int motorMin = 370 ;
const int motorMax = 470 ;

PCA9685 *pca9685 = new PCA9685();
int go = GO_MID;
int turn = TURN_MID;

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn;
}


void callBack (const geometry_msgs::Twist &msg)
{
    //check it's RESET 
    if (msg.linear.x == RESET_CODE and msg.angular.z == RESET_CODE)
    {
	go = GO_MID;
	turn = TURN_MID; 
	ROS_INFO(" ------- pca9685 RESET ------- ");
    }
    else 
    {
        go += msg.linear.x;
        turn += msg.angular.z;
	
        //check it's in the legal range
        if (turn > TURN_MAX) turn = TURN_MAX;
        if (turn < TURN_MIN) turn = TURN_MIN;
        if (go > GO_MAX) go = GO_MAX;
    }
        ROS_INFO("GO : %i" , go);
        ROS_INFO("turn: %i"  , turn);
}

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"cmd_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_vel",1000, &callBack);
    //get param i2c port
    std::string temp_name = "/dev/i2c-0"; //default to be i2c-0
    n.getParam("/pca9685_pub/port",temp_name);

    std::cout << "temp_name" << temp_name << std::endl;

    int err = pca9685->openPCA9685(temp_name);
    if (err < 0)
    {
        printf("Error: %d", pca9685->error);
    } 
    else 
    {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
	pca9685->setAllPWM(0,0) ;
        pca9685->reset();
        pca9685->setPWMFrequency(60);
	
	//Let servo say hello
        pca9685->setPWM(2,0, map(TURN_MID + 50 ,TURN_MIN, TURN_MAX ,servoMin, servoMax));
        sleep(1);
        pca9685->setPWM(2,0, map(TURN_MID - 50 ,TURN_MIN, TURN_MAX ,servoMin, servoMax));
        sleep(1);
        pca9685->setPWM(2,0, map(TURN_MID + 50 ,TURN_MIN, TURN_MAX ,servoMin, servoMax));
        sleep(1);
        pca9685->setPWM(2,0, map(TURN_MID - 50 ,TURN_MIN, TURN_MAX ,servoMin, servoMax));
        sleep(1);

	while(ros::ok())
	{
	    ros::spinOnce();
	    pca9685->setPWM(0,0, map(go,  GO_MID ,  GO_MAX   ,motorMin, motorMax));
	    pca9685->setPWM(2,0, map(turn,TURN_MIN, TURN_MAX ,servoMin, servoMax));
	}
	/*
        while(pca9685->error >= 0)
	{
            pca9685->setPWM(0,0,servoMin) ;
            pca9685->setPWM(1,0,servoMin) ;
            sleep(2) ;
            pca9685->setPWM(0,0,servoMax) ;
            pca9685->setPWM(1,0,map(90,0,180,servoMin, servoMax)) ;
            sleep(2) ;
        }*/
    }
    pca9685->closePCA9685();
    return 0;
}
