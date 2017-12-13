/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "pp_controller.h"
#include <std_msgs/Bool.h>

L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    //Distance between two wheels
    pn.param("L", L, 0.325);
    //pn.param("Lrv", Lrv, 10.0);
    //base speed of racecar m/s
    pn.param("Vcmd", Vcmd, 1.0);
    //distance from rear wheels to base_link point.
    pn.param("lfw", lfw, 0.1625);
    //pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    //the constant speed of racecar goes.
    pn.param("baseSpeed", baseSpeed, 1600);
    //degree
    pn.param("baseAngle", baseAngle, 90.0);
    pn.param("Vexp" , Vexp, 0.5);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    pub_reach = n_.advertise<std_msgs::Bool>("reach", 1);
    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lfw = goalRadius =  0.5;//getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    seq = 0;
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;

    //Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
}

void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


// if get a new goal point.
void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
    	ROS_INFO("GOAL_Received!!");
		goal_reached = false;
        seq = 0;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

//return  car_pose yaw
double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double dummy,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(dummy,dummy, yaw);

    return yaw;
}

bool L1Controller::isGoodWay(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
   /*check is forward waypt*/
    float dx = wayPt.x - carPose.position.x;
    float dy = wayPt.y - carPose.position.y;
    double yaw = getYawFromPose(carPose);

    float px = cos(yaw)*dx + sin(yaw)*dy;

    if(px <= 0) /*is NOT Forward WayPt*/
        return false;

    /*check is far enough to go*/
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else //(dist >= Lfw)
        return true;
}



double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    geometry_msgs::Point forwardPt;
    foundForwardPt = false;

    /*find the vaild local goal for this moment car_pose. */

    if(!goal_reached){
        for(int i = seq; i < map_path.poses.size(); i++)
        //while(1)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                
                if (isGoodWay(odom_path_wayPt, carPose)) 
                {
                  //ROS_INFO("seq = %i", seq);
                  /*this is the output*/
                  forwardPt = odom_path_wayPt;
                  foundForwardPt = true;
                  seq = i;
                  break;
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }// end for loop.
        
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    
    /*calculate the output by carPose and forwardPt*/
    double yaw = getYawFromPose(carPose);
    double px = cos(yaw)*(forwardPt.x - carPose_pos.x)+ 
                sin(yaw)*(forwardPt.y - carPose_pos.y);
    double py =-sin(yaw)*(forwardPt.x - carPose_pos.x)+ 
                cos(yaw)*(forwardPt.y - carPose_pos.y);

    /*getEta*/
    return atan2(py, px);
}

double L1Controller::getCar2GoalDist()
{
    double dx = odom_goal_pos.x - odom.pose.pose.position.x;
    double dy = odom_goal_pos.y - odom.pose.pose.position.y;
    return sqrt(dx*dx + dy*dy); // dist to goal
}

//depend on the Vcmd give differernt Lfw
double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}


//Speed control, constant speed don't need this.

double L1Controller::getGasInput(const float& current_steer, const double& current_v)
{
    double u = (Vexp - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    
    //u = - abs(current_steer - baseAngle) * 1;
    return u;
}


//this timer is to check if racecar Reach Goal.
void L1Controller::goalReachingCB(const ros::TimerEvent&)
{
    if(goal_received and getCar2GoalDist() < goalRadius)
    {
      goal_reached = true;
      goal_received = false;
      seq = 0;
      std_msgs::Bool b;
      b.data = true;
      pub_reach.publish(b);
      ROS_INFO("Goal Reached !");
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = odom.pose.pose;
    double current_v = odom.twist.twist.linear.x;
    //geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = baseAngle;

    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);  
        if(foundForwardPt)
        {
            cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain;
            /*Estimate Gas Input*/
            if(!goal_reached)
            {
                double u = getGasInput(cmd_vel.angular.z, current_v);
                cmd_vel.linear.x = 1620 + u;
                ROS_INFO("\nGas = %.2f\nSteering angle = %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
            }
        }
    }
    /*output of this node*/
    pub_.publish(cmd_vel);
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "pp_controller");
    L1Controller controller;

    ros::spin();
    return 0;
}
