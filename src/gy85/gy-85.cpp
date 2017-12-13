#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "gy-85.h"
#include <termios.h>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

const double d2g = M_PI/180;


GY85::GY85(const char *i2c_bus) : file(-1), offset_x(0), offset_y(0), offset_z(0) {
    file = open(i2c_bus, O_RDWR);
    if (file<0) {
        std::cerr << "Can not open device " << i2c_bus << std::endl;
        return;
    }

    // init the magnetometer
    if (ioctl(file, I2C_SLAVE, HMC5883) < 0) {
        std::cerr << "Can not set device address " << HMC5883 << std::endl;
        return;
    }
    if (i2c_smbus_write_byte_data(file, 0x02, 0) < 0) {
        std::cerr << "i2c write failed" << std::endl; // mode register, continuous measurement mode
    }
    if (i2c_smbus_write_byte_data(file, 0x01, 0x20) < 0) {
        std::cerr << "i2c write failed" << std::endl; // configuration register B, set range to +/-1.3Ga (gain 1090)
    }

    // init the accelerometer
    if (ioctl(file, I2C_SLAVE, ADXL345) < 0) {
        std::cerr << "Can not set device address " << ADXL345 << std::endl;
        return;
    }
    if (i2c_smbus_write_byte_data(file, 0x2d, 0x08) < 0) { // power up and enable measurements
        std::cerr << "i2c write failed" << std::endl;
    }
    if (i2c_smbus_write_byte_data(file, 0x31, 0x00) < 0) { // set range to +/-2g
        std::cerr << "i2c write failed" << std::endl;
    }
    
    // init the gyroscope
    if (ioctl(file, I2C_SLAVE, ITG3200) < 0) {
        std::cerr << "Can not set device address " << ITG3200 << std::endl;
        return;
    }
    // the i2c register should be assign by what ?????
    if (i2c_smbus_write_byte_data(file, 0x15, 0x07) < 0) { 
        std::cerr << "i2c write failed" << std::endl;
    }
    if (i2c_smbus_write_byte_data(file, 0x16, 0x1e) < 0) {
        std::cerr << "i2c write failed" << std::endl;
    }
/*
    if (i2c_smbus_write_byte_data(file, 0x17, 0x00) < 0) {
        std::cerr << "i2c write failed" << std::endl;
    }
    if (i2c_smbus_write_byte_data(file, 0x3e, 0x00) < 0) {
        std::cerr << "i2c write failed" << std::endl;
    }
*/
}

GY85::~GY85() {
    if (file>=0) close(file);
}

void GY85::set_magnetometer_offset(float ox, float oy, float oz) {
    offset_x = ox;
    offset_y = oy;
    offset_z = oz;
}

bool GY85::get_heading(float &heading) {
    float x, y, z;
    bool res = read_magnetometer(x, y, z);
    if (!res) return false;

    heading = atan2(y, x);
    if (heading < 0     ) heading += 2*M_PI;
    if (heading > 2*M_PI) heading -= 2*M_PI;
    heading = heading*180/M_PI;
    return true;
}

bool GY85::read_magnetometer(float &x, float &y, float &z) {
    if (ioctl(file, I2C_SLAVE, HMC5883) < 0) {
        std::cerr << "Can not set device address " << HMC5883 << std::endl;
        return false;
    }

    __u8 buf[6] = {0};
    i2c_smbus_read_i2c_block_data(file, 0x03, 6, buf);

    unsigned short ux = ((unsigned short)buf[0] << 8) | ((unsigned short)buf[1]);
    unsigned short uz = ((unsigned short)buf[2] << 8) | ((unsigned short)buf[3]);
    unsigned short uy = ((unsigned short)buf[4] << 8) | ((unsigned short)buf[5]);

    short xraw = *(short *)(void *)&ux;
    short yraw = *(short *)(void *)&uy;
    short zraw = *(short *)(void *)&uz;

    x = float(xraw)/1090. - offset_x;
    y = float(yraw)/1090. - offset_y;
    z = float(zraw)/1090. - offset_z;

    return true;
}

bool GY85::read_accelerometer(float &x, float &y, float &z) {
    if (ioctl(file, I2C_SLAVE, ADXL345) < 0) {
        std::cerr << "Can not set device address " << ADXL345 << std::endl;
        return false;
    }
    __u8 buf[6] = {0};
    i2c_smbus_read_i2c_block_data(file, 0x32, 6, buf);
    unsigned short ux = ((unsigned short)buf[1] << 8) | ((unsigned short)buf[0]);
    unsigned short uy = ((unsigned short)buf[3] << 8) | ((unsigned short)buf[2]);
    unsigned short uz = ((unsigned short)buf[5] << 8) | ((unsigned short)buf[4]);

    short xraw = *(short *)(void *)&ux;
    short yraw = *(short *)(void *)&uy;
    short zraw = *(short *)(void *)&uz;

    x = xraw/256.; // +/- 2G range for 10 bit resolution
    y = yraw/256.;
    z = zraw/256.;

    return true;
}

bool GY85::read_gyroscope(float &x, float &y, float &z, float &te) 
{
    if (ioctl(file, I2C_SLAVE, ITG3200) < 0) {
        std::cerr << "Can not set device address " << ITG3200 << std::endl;
        return false;
    }
    __u8 buf[8] = {0};
    i2c_smbus_read_i2c_block_data(file, 0x1B, 8, buf);

    unsigned short ux = ((unsigned short)buf[2] << 8) | ((unsigned short)buf[3]);
    unsigned short uy = ((unsigned short)buf[4] << 8) | ((unsigned short)buf[5]);
    unsigned short uz = ((unsigned short)buf[6] << 8) | ((unsigned short)buf[7]);
    unsigned short ut = ((unsigned short)buf[0] << 8) | ((unsigned short)buf[1]);
   

    short xraw = *(short *)(void *)&ux;
    short yraw = *(short *)(void *)&uy;
    short zraw = *(short *)(void *)&uz;
    short traw = *(short *)(void *)&ut;
    
    int xoff = 53;
    int yoff = 2;
    int zoff = 6;

    x = (xraw + xoff) / 14.375;
    y = (yraw + yoff) / 14.375;
    z = (zraw + zoff) / 14.375;
    te = 35 + (traw + 13200) / 280;
    



    return true;
}


//************************
//*********imuPub*********
//************************


void imuPub::reset()
{
	ROS_INFO(" ----------------IMU orientation RESET !!! -----------");
    roll = 0;
    pitch = 0;
    yaw = 0;
    la_gx = 0;
    la_gy = 0;
    la_gz = 0;
	
	//calibrate
    ROS_INFO("gy85 calibrating : %i" ,init_loop);
    zBias = _calibration(init_loop);
}

double imuPub::_calibration(const int &init_loop)
{
    int count = 0;
    double bias = 0;
    ros::Rate temp_rate(sRateHz);
    ROS_INFO("gy85 start calibration");
    while (ros::ok())
    {
    	readRaw();
    	bias += gz;
    	count++;
    	if (count > init_loop)
    	{
    	    bias /= init_loop;
            ROS_INFO("gy85 calibrate FINISHED");
    		ROS_INFO("gy85 angular velecity Z bias = %lf", bias);
    	    break;
	    }
	    temp_rate.sleep();
    }
    return bias;
}

void imuPub::readRaw()
{
    //get inertial data
    gy85.read_accelerometer(ax,ay,az);
    gy85.read_gyroscope(gx,gy,gz,te);
    gy85.get_heading(heading);

    //get time
    time = ros::Time::now();
    dt = (time - la_time).toSec();
}

void imuPub::getRPY()
{  
	//if (abs(gx) > threshold)
    roll  += (la_gx + gx)*dt/2;
    //if (abs(gy) > threshold)
    pitch += (la_gy + gy)*dt/2;
    //if (abs(gz) > threshold) 
    yaw   += (la_gz + gz - 2*zBias)*dt/2;
}

void imuPub::this2La()
{
	la_time = time;
	la_gx = gx;
	la_gy = gy;
	la_gz = gz;
}


void imuPub::intputData()
{
	// input imu frame
	msg.header.frame_id = frame;

	//input linear_acc
	msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

	//input angular_velocity
    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;

	//input time
	msg.header.stamp = time;
	//input YAW
	//obs_mat.setEulerYPR(yaw*d2g,pitch*d2g,roll*d2g);
    tf::Matrix3x3 obs_mat;
    tf::Quaternion q_tf;
	obs_mat.setEulerYPR(yaw*d2g, 0 ,0);
	obs_mat.getRotation(q_tf);
	msg.orientation.x = q_tf.getX();
	msg.orientation.y = q_tf.getY();
	msg.orientation.z = q_tf.getZ();
	msg.orientation.w = q_tf.getW();
}

void imuPub::setParams(int init_loop, int sRateHz, std::string port, std::string frame)
{
    this->init_loop = init_loop;
    this->sRateHz = sRateHz;
    this->port = port;
    this->frame = frame;
}

