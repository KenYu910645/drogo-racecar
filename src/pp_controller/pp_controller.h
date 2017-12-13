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
#include <cmath>

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
    public:
        L1Controller();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        bool isGoodWay(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta)
        {return -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/M_PI);}
        double getGasInput(const float& current_steer, const double& current_v);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub;
        ros::Publisher pub_, marker_pub, pub_reach;
        //timer1 for controllerCB , timer2 for goalReaching CB
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle;
        //final result of this program
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
        //read-time car pose odom.
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path;
        
        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
        double Gas_gain, baseAngle, Angle_gain, goalRadius, Vexp;
        int controller_freq, baseSpeed;
        int seq;
        bool foundForwardPt, goal_received, goal_reached;
        //get real-time odom from rf2o
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
             {odom = *odomMsg;}
        //get path msg from move base
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
             {map_path = *pathMsg;}
        //get gool from move base
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        /*timer callback*/
        //check if reach the goal
        void goalReachingCB(const ros::TimerEvent&);
        //output steer and throttle.
        void controlLoopCB(const ros::TimerEvent&);

}; // end of class

