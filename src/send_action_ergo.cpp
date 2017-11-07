// This node is a client node used to send the displacement of the robot as a ROS Action
// Created by King's College London and Queen Mary University of London, 2017.

// Include all the needed libraries. The non-standard library needed by the code is the libjsoncpp for JSON Array definition and fourbythree_msgs. See Readme file for download instructions.
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

// Initialize the pose message
geometry_msgs::Pose ergo_pose;
// Initialize the client simple action using the fourbythree_msgs data type
typedef actionlib::SimpleActionClient<fourbythree_msgs::ExecuteInstructionAction> Client;
// Initialize variable 'goal' to send stiffness value to the server
fourbythree_msgs::ExecuteInstructionGoal goal;
// Initialize the ergonomics variable in the form of JSON String
Json::Value ergo_str;
// Initialize the flag variable
unsigned char flag_action;

// This function is used to get the target pose
void get_target(const geometry_msgs::Pose::ConstPtr& msg)
{
  ergo_pose = *msg;
  // Set the flag, indicating that the data has been received
  flag_action = 1;
}

// This is the main function
int main(int argc, char** argv)
{
  // Initialize ROS Node
  ros::init(argc, argv, "send_action_ergo");
  ros::NodeHandle n;

  // Used to get the robot's tip displacement
  ros::Subscriber target_sub = n.subscribe("/fourbythree_topics/ergonomics/baxter_target", 10, get_target);
  // Used to publish status whether the robot has finished the task
  ros::Publisher task_finish_pub = n.advertise<std_msgs::Int32>("/fourbythree_topics/ergonomics/task_finish_flag",10);
  // Initialize the flag representing the task status
  std_msgs::Int32 task_flag;
  task_flag.data = 0;

  // Define the type of data as 'ERGONOMICS' to be sent to the server
  goal.type = "ERGONOMICS";
  // Initialize the client variable under the name 'execute_sm_instruction'
  Client client("execute_sm_instruction", true);
  // Reset the flag
  flag_action = 0;
  // Printing info
  ROS_INFO("A");
  // Wait until Server under the name 'execute_sm_instruction' is ready
  client.waitForServer();
  // Print status indicating that the server is ready
  ROS_INFO("B");

  while(ros::ok())
  {
    // Check whether pose data has been received
    if(flag_action == 1)
    {
      // Initialize the orientation variables
      double roll, pitch, yaw;

      // Check whether the pose includes displacement of orientation
      if(ergo_pose.orientation.x!=0 && ergo_pose.orientation.y!=0 && ergo_pose.orientation.z!=0 && ergo_pose.orientation.w!=0)
      {
        // Initialize the quaternion
        tf::Quaternion q(ergo_pose.orientation.x, ergo_pose.orientation.y, ergo_pose.orientation.z, ergo_pose.orientation.w);
        // Transform quaternion into rotation matrix
        tf::Matrix3x3 m(q);
        // Transform rotation matrix into RPY angles
        m.getRPY(roll, pitch, yaw);
      }
      // If no orientation detected, set RPY angles into zero
      else
      {
        roll = 0; pitch = 0; yaw = 0;
      }

      // Add the pose value into JSON array
      ergo_str["x"] = ergo_pose.position.z;
      ergo_str["y"] = -ergo_pose.position.y;
      ergo_str["z"] = ergo_pose.position.x;
      ergo_str["roll"] = roll;
      ergo_str["pitch"] = pitch;
      ergo_str["yaw"] = yaw;

      // Define the string stream variable used to change the JSON array into string format
      std::stringstream ss;
      // Transformed the JSON Array into string format
      ss << ergo_str;
      // Add the string in JSON format to goal parameter to be sent
      goal.parameters = ss.str();

      // Send the pose values written as string in JSON format to server
      client.sendGoal(goal);
      // Set the flag so that the algorithm wait for the robot to finish the task
      task_flag.data = 1;
      // Send the flag
      task_finish_pub.publish(task_flag);

      // Wait for response from server
      client.waitForResult();
      // Check whether the data transfer to server is succeeded
      if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        // If so, reset the flag
        flag_action = 0;
        task_flag.data = 0;
      }
    }
    // Send the flag
    task_finish_pub.publish(task_flag);
    // Spin the loop once
    ros::spinOnce();
  }
  return 0;
}
