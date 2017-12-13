#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <cmath>

using namespace std;
using namespace geometry_msgs;

#define SAMPLE_COUNT 360
#define totalPoint 45

class Lidar
{
  public:
    Lidar() {reset();}
    void setRan_score();
    void setYaw_score(bool isForward);
    void setCombo_score();
    void reset();
    int  getBestAng();
  private:
    double  ran_score[SAMPLE_COUNT];
    double  yaw_score[SAMPLE_COUNT];
    double  combo_score[SAMPLE_COUNT];
};

Point setXYwithRth(double r, double theta);

class Vector 
{
  public:
    Vector(){reset();}
	void reset ()
	{
      eta  = 0;
      a = 0;//slop
      b = 0;
      c = 0;
      dis = 0;
	  validPoint = 0;
      //Marks
      vline.header.frame_id = "laser";
      vline.ns = "Markers";
      vline.action = visualization_msgs::Marker::ADD;
      vline.type = visualization_msgs::Marker::LINE_STRIP;
      vline.scale.x = 0.1;
      vline.pose.orientation.w = 1;
	}


    void   setPoints(int validPoint, double r , double theta)
	{
	  points[validPoint] = setXYwithRth(r, theta);
	}
    double getSlop() {return a;}
    double getDis() {return dis;} 
    double getEta();
    Point getStartPoint(){return points[0];}
    Point getEndPoint(){return points[validPoint-1];}
    void addOneValid(){validPoint++;}
	int getValidPoint(){return validPoint;}

    //Mark is public member.
    visualization_msgs::Marker vline;
  private:
    //helper function
    double getXavg();
    double getYavg();

    //data member
    double eta;
    // ax + by + c = 0 ,  'a' is slop.
    double a, b, c;
    double dis;
	int validPoint;
    Point points[totalPoint];
};

class Marker
{
  public:
    Marker(int id, double r , double g , double b , double a)
    {
      vline.header.frame_id = "laser";
      vline.ns = "Markers";
      vline.action = visualization_msgs::Marker::ADD;
      vline.type = visualization_msgs::Marker::LINE_STRIP;
      vline.scale.x = 0.1;
      vline.pose.orientation.w = 1;
      vline.id = id;
      //color
      vline.color.r = r;
      vline.color.g = g;
      vline.color.b = b;
      vline.color.a = a;
    }
    void set2Point(double x1, double y1, double x2, double y2);
    visualization_msgs::Marker vline;
};

