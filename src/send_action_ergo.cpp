
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <actionlib/client/simple_action_client.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <string>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <sstream>

std_msgs::Float64MultiArray ergo_pose;
typedef actionlib::SimpleActionClient<fourbythree_msgs::ExecuteInstructionAction> Client;
fourbythree_msgs::ExecuteInstructionGoal goal;
Json::Value ergo_str;
// This function is used to get the stiffness vector
void get_target(const geometry_msgs::Pose::ConstPtr& msg)
{
  ergo_pose = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_action_ergo");
  ros::NodeHandle n;

  // Used to get the stiffness vector
  ros::Subscriber stiff_sub = n.subscribe("/fourbythree_topics/stiffness/stiffness_vector", 10, get_stiff);
  unsigned char flag_action = 0;
  stiffness.data.clear();
  for(unsigned char i=0; i<3; i++)
  {
    stiffness.data.push_back(0);
  }

  goal.type = "STIFFNESS";
  Client client("execute_sm_instruction", true);
  client.waitForServer();

  while(flag_action == 0 and ros::ok())
  {

    flag_action = 1;

    stiff_str.clear();
    for(unsigned char i=0; i<3; i++)
    {
      stiff_str.append(stiffness.data[i]);
    }

    std::stringstream ss;
    ss << stiff_str;
    goal.parameters = ss.str();

    client.sendGoal(goal);

    client.waitForResult();
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      flag_action = 0;
    }

    ros::spinOnce();
  }
  return 0;
}
