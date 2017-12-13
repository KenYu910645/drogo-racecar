#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cmath>
#include "gy-85.h"
#include <ros/ros.h>
#include <string>

//const double threshold = 0.3;
const double d2g = M_PI/180;
 
//global imuPub class e
imuPub e;

void callBack (const geometry_msgs::Twist &msg)
{
    if (msg.linear.x == 666 and msg.angular.z == 666)
		e.reset();
}

int main (int argc, char** argv)
{
  	ros::init(argc,argv,"imu_auto_lucky");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callBack);

    //getParams
    int init_loop, sRateHz;
    std::string port = "/dev/i2c-1";
    std::string frame = "imu_link";
    n.param("/imu_auto_lucky/init_loop", init_loop, 300);
    n.param("/imu_auto_lucky/sRateHz", sRateHz, 100);
    n.getParam("/imu_auto_lucky/port", port);
    n.getParam("/imu_auto_lucky/imu_frame", frame);
    const char* device_name = port.c_str();
    
    //setParams
    e.setParams(init_loop, sRateHz, port, frame);

    ros::Rate rate(sRateHz);
    e.reset();

	while (ros::ok())
    {
		//listen to control node , check if "RESET"
		ros::spinOnce();
        
		e.readRaw();
		e.getRPY();
        e.intputData();
		//publish the msg
		pub.publish(e.getMsg());
        e.this2La();
		//wait for the loop end
		rate.sleep();
    }
    e.~imuPub();
	return 0;
}
