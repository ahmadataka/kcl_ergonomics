#include <ros/ros.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <actionlib/server/simple_action_server.h>
#include <iostream>

using namespace std;
typedef actionlib::SimpleActionServer<fourbythree_msgs::ExecuteInstructionAction> Server;

void execute(const fourbythree_msgs::ExecuteInstructionGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
  fourbythree_msgs::ExecuteInstructionGoal testing;
  testing = *goal;
  cout << testing.parameters << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "server_ergonomics");
  ros::NodeHandle n;
  Server server(n, "execute_sm_instruction", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
