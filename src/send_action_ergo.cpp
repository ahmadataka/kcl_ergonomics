
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"
#include <actionlib/client/simple_action_client.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <string>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <tf/transform_datatypes.h>

geometry_msgs::Pose ergo_pose;
typedef actionlib::SimpleActionClient<fourbythree_msgs::ExecuteInstructionAction> Client;
fourbythree_msgs::ExecuteInstructionGoal goal;
Json::Value ergo_str;
unsigned char flag_action;
// This function is used to get the stiffness vector
void get_target(const geometry_msgs::Pose::ConstPtr& msg)
{
  ergo_pose = *msg;
  flag_action = 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_action_ergo");
  ros::NodeHandle n;

  // Used to get the stiffness vector
  ros::Subscriber target_sub = n.subscribe("/fourbythree_topics/ergonomics/baxter_target", 10, get_target);
  ros::Publisher task_finish_pub = n.advertise<std_msgs::Int32>("/fourbythree_topics/ergonomics/task_finish_flag",10);
  std_msgs::Int32 task_flag;

  goal.type = "ERGONOMICS";
  Client client("execute_sm_instruction", true);
  flag_action = 0;
  ROS_INFO("A");
  client.waitForServer();
  ROS_INFO("B");
  while(ros::ok())
  {
    if(flag_action == 1)
    {
      tf::Quaternion q(ergo_pose.orientation.x, ergo_pose.orientation.y, ergo_pose.orientation.z, ergo_pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      ergo_str["x"] = ergo_pose.position.x;
      ergo_str["y"] = ergo_pose.position.y;
      ergo_str["z"] = ergo_pose.position.z;
      ergo_str["roll"] = roll;
      ergo_str["pitch"] = pitch;
      ergo_str["yaw"] = yaw;

      std::stringstream ss;
      ss << ergo_str;
      goal.parameters = ss.str();

      client.sendGoal(goal);

      client.waitForResult();
      if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        flag_action = 0;
        task_flag.data = 1;
        task_finish_pub.publish(task_flag);
      }
    }

    ros::spinOnce();
  }
  return 0;
}
