#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#define S 4

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isReach  = false;

void callBack (const std_msgs::Bool &msg)
{
  if (msg.data == true)
    isReach = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  ros::Subscriber sub = n.subscribe("reach", 1000, callBack);
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
  
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped simple_goal;
  
  geometry_msgs::Pose p[S];
  p[0].position.x = 1.02;
  p[0].position.y = 0.541;
  p[1].position.x = 1.32;
  p[1].position.y = 2.57;
  p[2].position.x = -0.6;
  p[2].position.y = 2.95;
  p[3].position.x = -1.14;
  p[3].position.y = 1.12;




  //at start point 
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();  
  goal.target_pose.pose.position = p[0].position;
  goal.target_pose.pose.orientation.w = 1;
  simple_goal.pose = goal.target_pose.pose;
  simple_goal.header = goal.target_pose.header;

  ROS_INFO("Sending goal at index 0 ");
  pub.publish(simple_goal);
  ac.sendGoal(goal);
  isReach = false;

  //we'll send a goal to the robot to move 1 meter forward
  int index = 0;
  int round = 0;
  while(ros::ok())
  {
    ros::spinOnce();
    
    if (isReach and 0 <= index and index < S )//at mid point.
    {
      ROS_INFO("I REACH GOAL at index %i!!!!!", index);
      
	  index++;
	  if (index >= S)
	  {
		  round++;
		  ROS_INFO("%ith round is over", round);
		  index = 0;
	  }

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();  
      goal.target_pose.pose.position = p[index].position;
      goal.target_pose.pose.orientation.w = 1;
      simple_goal.pose = goal.target_pose.pose;
      simple_goal.header = goal.target_pose.header;

      ROS_INFO("Sending goal at index %i ", index);
      pub.publish(simple_goal);
      ac.sendGoal(goal);
      isReach = false;
    }
  }
  ROS_INFO("navigation is over!!  bye bye ~~");
  return 0;
}
