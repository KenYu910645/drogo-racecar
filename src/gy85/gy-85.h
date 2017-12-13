#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#ifndef __GY_85_H__
#define __GY_85_H__

const int ADXL345 = 0x53; //address of Accelerometer
const int HMC5883 = 0x1E; //address of Magnetometer
const int ITG3200 = 0x68; //address of Gyro

class GY85 {
public:
    GY85(const char *i2c_bus);
    ~GY85();
    bool get_heading(float &h);
    bool read_magnetometer(float &x, float &y, float &z);   // returns Ga
    bool read_accelerometer(float &x, float &y, float &z);  // returns g
    bool read_gyroscope(float &x, float &y, float &z, float &te); 
    void set_magnetometer_offset(float ox, float oy, float oz);
private:
    int file;
    float offset_x;
    float offset_y;
    float offset_z;
};

class imuPub
{
    public:
		void reset();
		void this2La();
		void readRaw();
		void intputData();
        void getRPY();
        void setParams(int , int, std::string, std::string);
        sensor_msgs::Imu& getMsg() {return msg;}
		imuPub() : roll(0), pitch(0), yaw(0), la_gx(0), la_gy(0), la_gz(0), zBias(0),  gy85("/dev/i2c-1") {}
		~imuPub()
		{
			gy85.~GY85();
		}
    private:
		//param
		int init_loop, sRateHz;
		std::string port, frame;
		const char* device_name;
		//data
		double roll, pitch, yaw;
		float la_gx, la_gy, la_gz;
		float ax, ay, az, gx, gy, gz, te, heading;
		double zBias, dt;
		//time 
		ros::Time time, la_time;
		//message type
    	sensor_msgs::Imu msg;
		// gy85 object
		GY85 gy85;
		//helper funciton
		double _calibration(const int &init_loop);
};

#endif //__GY_85_H__

