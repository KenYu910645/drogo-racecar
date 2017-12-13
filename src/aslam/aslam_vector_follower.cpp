#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <cmath>
#include "aslam_vector_follower.h"

using namespace std;
using namespace geometry_msgs;

#define totalPoint 45
#define SAMPLE_COUNT 360
#define SCORE_RADIUS 2
#define RECOVERY_RADIUS 0.3
#define RECOVERY_TIME 20
#define USE_RECOVERY 0
#define USE_VECTOR 0
#define USE_DEVIATE 1

const double d2g = M_PI/180;
const int LIDAR_MAX = 8; // m

//global array for scroing
double r[SAMPLE_COUNT] = {0};
//recovery count
int recovery_count = 0;
/*****************************/
/******Global function********/
/*****************************/

//transfrom R, theta  into X, Y coordinate
Point setXYwithRth (double r , double theta)
{
  geometry_msgs::Point ans;
  ans.x = -cos(d2g*theta)*r;
  ans.y = -sin(d2g*theta)*r;
  return ans;
}

bool isInf(double tmp)
{
  if (tmp == 1.0/0.0)
    return true;
  else 
    return false;
}

/*****************************/
/******Vector function********/
/*****************************/

double Vector::getEta()
{
  double avgX = getXavg();
  double avgY = getYavg();
  double num  = 0;
  double deno = 0;
  for (int i = 0 ; i < validPoint; i++)
  {
    double dx = points[i].x - avgX;
    double dy = points[i].y - avgY;
    deno += dx * dx;
    num  += dx * dy;
  }
  a = num/deno;//slop
 
  if(a < 0.02 and a > -0.02) // tan(-1~1)
    eta = 0;
  else 
    eta = atan2(num, deno) /d2g;
  
  // get ax + by + c = 0
  b = -1;
  c = -a*avgX +avgY ;
  //get distance between vector and (0,0)
  dis = abs(c)/sqrt(a*a + b*b); 
  
  return eta;
}

double Vector::getXavg()
{
  double ans = 0;
  for (int i = 0 ; i < validPoint; i++)
    ans += points[i].x;
  ans /= validPoint;
  return ans;
}

double Vector::getYavg()
{
  double ans = 0;
  for (int i = 0 ; i < validPoint; i++)
    ans += points[i].y;
  ans /= validPoint;
  return ans;
}

/*****************************/
/******Scoring function*******/
/*****************************/

void Lidar::setRan_score()
{
  for (int i = 0 ; i < SAMPLE_COUNT ; i++)
  {
    //if (r[i] == 1.0/0.0) //inf 
    if (r[i] >= SCORE_RADIUS or isInf(r[i]))
      ran_score[i] = 100;
    else 
      ran_score[i] = r[i]*(100/SCORE_RADIUS);
    
    //debug
    //ROS_INFO("[ %i ] = %2f",i ,ran_score[i]);
  }
}


void Lidar::setYaw_score(bool isForward)
{
  for(int i = 0; i < SAMPLE_COUNT; i++)
  {
    double raw;
    if (isForward)
      raw =   180 - abs(i - 180);
    else
      raw = -(180 - abs(i - 180));
    
    //map 'raw' to 0 ~ 100 score
    yaw_score[i] = raw * (100.0/90.0);

    if (isForward)
    {
      if (abs(i - 180) > 90)
        yaw_score[i] = -1000;
    }
    else 
    {
      if (abs(i - 180) < 90)
        yaw_score[i] = -1000;
    }
    //cout << "i = " << i << endl; 
    //cout << "yaw_score[i] = " << yaw_score[i] << endl;
  }
}

bool isNInRadius(double x)
{
  if (x > SCORE_RADIUS)
    return true;
  else 
    return false;
}

void Lidar::setCombo_score()
{
  //counter clockwise scoring
  for (int i = 1 ; i < SAMPLE_COUNT; i++)
  {
    if (isNInRadius(r[i]) and isNInRadius(r[i-1])) // inf 
      combo_score[i] = combo_score[i-1] + 1;
    else if (!isNInRadius(r[i]) and !isNInRadius(r[i-1]))
      combo_score[i] = combo_score[i-1] - 1;
    else // no combo!!
      combo_score[i] = 0;
  }
 
  //reverse scoring
  bool needModify = false;
  int midScore;
  combo_score[SAMPLE_COUNT - 1] = 0;
  for (int i = SAMPLE_COUNT-1; i >= 0 ;i--)
  {
    if (combo_score[i] == 0)
    {
      needModify = true;
      midScore = combo_score[i-1]/2; 
    }
    else if (combo_score[i] == midScore)
    {
      needModify = false;
    }
    else if (needModify)
    {
      if (combo_score[i+1] > 0) // inf combo
        combo_score[i] = combo_score[i+1] + 1;
      else if (combo_score[i+1] < 0) //obstacle combo
        combo_score[i] = combo_score[i+1] - 1;
      else // combo_score[i-1] == 0 
      {
        if (isNInRadius(r[i]))
          combo_score[i] = 1;
        else
          combo_score[i] = -1;
      }
    }
  }
 
  //linear enlarge
  double max = INT_MIN;
  for (int i = 0 ; i < SAMPLE_COUNT ;i++)
  {
    if (combo_score[i] > max)
      max = combo_score[i];
  }
  double ratio = 100.0 / max;
  for (int i = 0 ; i < SAMPLE_COUNT ;i++)
  {
    if (combo_score[i] > 0)
      combo_score[i] *= ratio;
  }

  //debug 
  //for (int i  = 0 ; i < SAMPLE_COUNT; i++)
    //cout << "[ "<< i << " ] = " << combo_score[i]  << endl; 
}

/*****************************/
/****** Lidar function********/
/*****************************/
int Lidar::getBestAng()
{
  double max = INT_MIN;
  int BestAng = -1;

  for(int i = 0 ; i < SAMPLE_COUNT ;i++)
  {
  	double totalScore = yaw_score[i] + ran_score[i] + combo_score[i];
  	if(totalScore > max)
  	{
  	  max = totalScore;
  	  BestAng = i;
  	}
  }
  ROS_INFO("[ %i ] totalScore = %.2f , ran_score = %.2f ,  yaw_score = %.2f , combo_score = %.2f ",BestAng , max, ran_score[BestAng], yaw_score[BestAng], combo_score[BestAng]);
  return BestAng;
}

void Lidar::reset()
{
  for (int i = 0 ; i < SAMPLE_COUNT;i++)
  {
    ran_score[i] = 0;
  	yaw_score[i] = 0;
  	combo_score[i] = 0;
  }
}

/*****************************/
/******CallBack function******/
/*****************************/
Vector Rvector;
Vector Lvector;

void callBack (const sensor_msgs::LaserScan &msg)
{
  for (int i = 0 ; i < SAMPLE_COUNT; i++)
    r[i] = msg.ranges[i];

  Rvector.reset();
  Lvector.reset();
  //get right vector
  for (int i = 90 ; i < 90+totalPoint; i++)
  {
	  if (!isInf(r[i]))
	  {
	    Rvector.setPoints(Rvector.getValidPoint(), msg.ranges[i], i);
	    Rvector.addOneValid();
	  }
  }
  
  //get left vector
  for (int i = 270 ; i > 270-totalPoint; i--)
  {   
	  if (!isInf(r[i])) //inf
	  {
	    Lvector.setPoints(Lvector.getValidPoint(), msg.ranges[i], i);
	    Lvector.addOneValid();
	  }
  }
}

/********************/
/******Markers*******/
/********************/

void Marker::set2Point(double x1, double y1, double x2, double y2)
{
  //clean the privise points
  vline.points.clear();

  geometry_msgs::Point tmp; 
  tmp.x = x1;
  tmp.y = y1;
  tmp.z = 0;
  vline.points.push_back(tmp);

  tmp.x = x2;
  tmp.y = y2;
  tmp.z = 0;
  vline.points.push_back(tmp);
}

bool needRecovery()
{
  for (int i = 0 ; i < SAMPLE_COUNT; i++)
  {
    if (r[i] < RECOVERY_RADIUS)//too close 
    {
      ROS_ERROR("Too close to obstacle, need to recovery!!");
      return true;
    }
  }
    return false;
}


/*****************/
/******MAIN*******/
/*****************/

int main (int argc, char** argv)
{
  // ROS basic 
  ros::init(argc, argv, "aslam_nav");
  ros::NodeHandle n;
  
  //Subscriber and Publisher
  ros::Subscriber sub = n.subscribe("scan", 1000 , callBack);
  ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1000);
  ros::Publisher  pub_mark = n.advertise<visualization_msgs::Marker>("/car_path", 1000);

  ros::Rate rate (20); 
 
  //Class declaration
  Lidar lidar;
  Marker rv(0, 1.0, 1.0, 0.0, 1.0);//Right Vector : yelllow
  Marker lv(1, 0.0, 1.0, 1.0, 1.0);//Left  Vector : light blue
  Marker sc(2, 0.0, 0.0, 1.0, 1.0);//Best score   : blue
  Marker de(3, 1.0, 0.0, 0.0, 1.0);//deviation    : red

  while (ros::ok())
  {
    //recieve data from previse topic
    ros::spinOnce();
    
    //analyse the scanning data, calculate.
    double rEta = Rvector.getEta();
    double lEta = Lvector.getEta();
  	//ROS_INFO("rEta = %.2f", rEta);
  	//ROS_INFO("lEta = %.2f", lEta);

    lidar.reset();
    lidar.setRan_score();
    #if USE_RECOVERY == 1
     if (needRecovery() or 
        (0 < recovery_count and recovery_count < RECOVERY_TIME))
     {
       lidar.setYaw_score(false);
       recovery_count++;
     }
     else 
       lidar.setYaw_score(true);
    #else
        lidar.setYaw_score(true);
    #endif
    
    lidar.setCombo_score();

    //get best Ang
    int score_bestAng = lidar.getBestAng();
    //ROS_INFO("score_bestAng = %i" , score_bestAng);
    // vector best ang
    double vector_bestAng = (rEta + lEta)/2;
    //ROS_INFO("vector_bestAng = %.2f" , vector_bestAng);
	  
    double Rdis = Rvector.getDis();
    double Ldis = Lvector.getDis();
    double roadDis = Rdis + Ldis;
	  //ROS_INFO("roadDis = %.2f", roadDis);
    double deviate_ang = ((Ldis - Rdis)/roadDis) * 30;
    
    //if vector is not reliable to follow
    //switch to score navigation.
    bool isVector  = true;
    bool isDeviate = false;
    
    if (abs(rEta - lEta) > 60)
      isVector = false;
    if (abs(Rdis - Ldis) > roadDis/8)
      isDeviate = true;
    //need recovery
    if (recovery_count != 0)
      isVector = false;
    if (recovery_count >= RECOVERY_TIME)
      recovery_count = 0;
    #if USE_VECTOR==0
      isVector = false;
    #endif
    #if USE_DEVIATE==0
      isDeviate = false;
    #endif
      
    /********SETTING MARKERS ********/

    if (isVector)
    {
      if (isDeviate)
      {
        rv.vline.color.a = 1.0;
        lv.vline.color.a = 1.0;
        sc.vline.color.a = 0.3;
        de.vline.color.a = 1.0;
      }
      else 
      {
        rv.vline.color.a = 1.0;
        lv.vline.color.a = 1.0;
        sc.vline.color.a = 0.3;
        de.vline.color.a = 0.3;
      }
    }
    else// !isVector 
    {
      if (isDeviate)
      {
        rv.vline.color.a = 0.3;
        lv.vline.color.a = 0.3;
        sc.vline.color.a = 1.0; 
        de.vline.color.a = 1.0;
      }
      else 
      {
        rv.vline.color.a = 0.3;
        lv.vline.color.a = 0.3;
        sc.vline.color.a = 1.0; 
        de.vline.color.a = 0.3;
      }
    }

    //R vector marker
    rv.set2Point(
        Rvector.getStartPoint().x , 
        Rvector.getStartPoint().y , 
        Rvector.getStartPoint().x + cos(rEta*d2g)*2 ,
        Rvector.getStartPoint().y + sin(rEta*d2g)*2 
        );
  	pub_mark.publish(rv.vline);

    //L vector marker
    lv.set2Point(
        Lvector.getStartPoint().x , 
        Lvector.getStartPoint().y , 
        Lvector.getStartPoint().x + cos(lEta*d2g)*2 ,
        Lvector.getStartPoint().y + sin(lEta*d2g)*2 
        );
  	pub_mark.publish(lv.vline);
    
    //Score Marker
    sc.set2Point(
        0 , 
        0 , 
        setXYwithRth(SCORE_RADIUS, score_bestAng).x ,
        setXYwithRth(SCORE_RADIUS, score_bestAng).y 
        );
  	pub_mark.publish(sc.vline);

    //deviation Marker
    de.set2Point(
        0 , 
        0 , 
        setXYwithRth(1, deviate_ang+180).x ,
        setXYwithRth(1, deviate_ang+180).y 
        );
  	pub_mark.publish(de.vline);


    //get cmd_vel
	  geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1540;
    if (isVector)
    {
      if (isDeviate)
        cmd_vel.angular.z = vector_bestAng + deviate_ang + 90;
      else 
        cmd_vel.angular.z = vector_bestAng + 90;
    }
    else 
    {
      if (isDeviate)
        cmd_vel.angular.z = (score_bestAng-180)  + 90 + deviate_ang*0.5;
      else 
        cmd_vel.angular.z = (score_bestAng-180)  + 90;
    }

    
    //post-process
    pub.publish(cmd_vel);
    rate.sleep();
  }
  return 0;
}
