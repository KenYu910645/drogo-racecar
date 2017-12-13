#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

double go = 0;
double trun = 0 ;

void callBack_angular(const geometry_msgs::Twist &msg)
{
	trun = msg.angular.z;
}

void callBack_linear(const geometry_msgs::Twist &msg)
{
	go = msg.linear.x;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "fuse_cmd_vel");
	ros::NodeHandle n;
	ros::Subscriber sub_angular = n.subscribe("/fuse/cmd_vel_angular", 1000, callBack_angular);
	ros::Subscriber sub_linear  = n.subscribe("/fuse/cmd_vel_linear",  1000, callBack_linear);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1000);
	ros::Rate rate(20);

	while (ros::ok())
	{
		geometry_msgs::Twist msg;

		ros::spinOnce();

		msg.linear.x = go;
		msg.angular.z = trun;

		//post-process
		pub.publish(msg);
		rate.sleep();
	}
}

