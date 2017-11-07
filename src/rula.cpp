// The code is used to produce the Rula Score from the human's joint angle.
// Created by King's College London and Queen Mary University of London, 2017.

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <math.h>

using namespace std;

// Initialize the rate
double freq = 40.0;
double dt = 1.0/freq;

// Define all the joint angles
double left_alpha, left_theta, left_wrist_1, left_wrist_2, left_gamma, left_phi;
// Define the row and column
int row,col;
// Define the RULA and individual score
int upper_arm_score, lower_arm_score, wrist_score, wrist_twist_score;
// Define the flag variables used as indicator for abduction and lateral movement
int abduct_flag, lateral_flag;

// This function is used to receive the human's joint angle
void get_angle_left(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Read the subscibed data
  std_msgs::Float64MultiArray pathv;
  pathv = *msg;
  // The human's joint angle is as follows
  // 1. Theta: lower arm Sagital Angle
  // 2. Alpha: upper arm Sagital Angle
  // 3. Gamma: upper arm Coronal angle
  // 4. Phi: lower arm Coronal angle

  left_theta  = pathv.data[0];
  left_alpha = pathv.data[1];
  left_wrist_1 = pathv.data[2];
  left_wrist_2 = pathv.data[3];
  left_gamma = pathv.data[4];
  left_phi = pathv.data[5];
}

// Define the rula_score for all possibility of joint angles
int rulatable[144] = {1,2,2,2,2,3,3,3,2,2,2,2,3,3,3,3,2,3,3,3,3,3,4,4,2,3,3,3,3,4,4,4,3,3,3,3,3,4,4,4,3,4,4,4,4,4,5,5,3,3,4,4,4,4,5,5,3,4,4,4,4,4,5,5,4,4,4,4,4,5,5,5,4,4,4,4,4,5,5,5,4,4,4,4,4,5,5,5,4,4,4,5,5,5,6,6,5,5,5,5,5,6,6,7,5,6,6,6,6,7,7,7,6,6,6,7,7,7,7,8,7,7,7,7,7,8,8,9,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,9};

// This function is used to get the rula score
int rula(double a, double b, double c, double d, double e, double f)
{
  // Define variable 'index' used to take the correct rula score from the table and 'ergo' as the rula score
  int index, ergo;

  //Step 1: Upper arm score
  if (a>=-10 && a<20) {upper_arm_score=1;}
  else if (a<-10){upper_arm_score=2;}
  else if (a>=20 && a<45){upper_arm_score=2;}
  else if (a>=45 && a<90) upper_arm_score=3;
  else if (a>=90){upper_arm_score=4;}
  else {upper_arm_score=0;}

  //Added to upper arm score when Abduction is detected
  if(e>25)
  {
    upper_arm_score = upper_arm_score + 1;
    abduct_flag = 1;
  }
  else abduct_flag = 0;

  //Step 2: Lower arm score
  if (b>=50 && b<100) {lower_arm_score=1;}
  if (b<50 || b>=100) {lower_arm_score=2;}

  // Added to lower arm score when lateral is detected
  if(f>25 || f<-25)
  {
    lower_arm_score = lower_arm_score + 1;
    lateral_flag = 1;
  }
  else lateral_flag = 0;

  //Step 3: Wrist Score
  if ((c>=-40 && c<-10) || (c<40 && c>=10)) wrist_score=1;
  else if (c>=40 || c<-40) wrist_score=2;
  else if(c>=-10 && c<10) wrist_score=1;

  // Step 4: The wrist twist score is set to 1
  wrist_twist_score = 1;

  // Step 5: Compute the overall Rula Score
  col = (wrist_score)*2 - (2-wrist_twist_score);
  row = upper_arm_score*3 - (3-lower_arm_score);
  index = (row-1)*8 + col-1;
  ergo = rulatable[index];

  // Return the Rula Score
  return ergo;
}

// Main function
int
main(int argc, char** argv)
  {
    // Initialize the ROS Node
    ros::init(argc,argv, "rula");
    ros::NodeHandle n;

    // Set the frequency
    ros::Rate loop_rate(freq);

    // Define the array to be published
    std_msgs::Float64MultiArray rula_score, upper, lower, wrist_score_array, wrist_twist, total_score;

    // Define the ROS Publisher
    // Used to publish RULA score
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/fourbythree_topics/ergonomics/rula_score",10);
    // Used to publish all scores
    ros::Publisher pub_total = n.advertise<std_msgs::Float64MultiArray>("/fourbythree_topics/ergonomics/total_score",10);
    // Used to publish upper arm score
    ros::Publisher pub_upper = n.advertise<std_msgs::Float64MultiArray>("/fourbythree_topics/ergonomics/upper_arm_score",10);
    // Used to publish lower arm score
    ros::Publisher pub_lower = n.advertise<std_msgs::Float64MultiArray>("/fourbythree_topics/ergonomics/lower_arm_score",10);
    // Used to publish wrist score
    ros::Publisher pub_wrist = n.advertise<std_msgs::Float64MultiArray>("/fourbythree_topics/ergonomics/wrist_score",10);
    // Used to publish wrist twist score
    ros::Publisher pub_wrist_twist = n.advertise<std_msgs::Float64MultiArray>("/fourbythree_topics/ergonomics/wrist_twist_score",10);
    // Used to publish the abduction status
    ros::Publisher abduct_pub = n.advertise<std_msgs::Int32MultiArray>("/fourbythree_topics/ergonomics/abduction_status",10);
    // Used to publish the lateral status
    ros::Publisher lateral_pub = n.advertise<std_msgs::Int32MultiArray>("/fourbythree_topics/ergonomics/lateral_status",10);

    // Define the ROS Subscriber
    // Used to subscribe to human's joint angle of the left arm
    ros::Subscriber left_angle = n.subscribe("/fourbythree_topics/ergonomics/left_human_angle", 10, get_angle_left);

    // Define the variables used to send the abduction and lateral status
    std_msgs::Int32MultiArray abduct_sent;
    std_msgs::Int32MultiArray lateral_sent;

    while(ros::ok())
    {
      // Clear and add the RULA Score from the human's joint angle
      rula_score.data.clear();
      rula_score.data.push_back(rula(left_alpha, left_theta, left_wrist_1,left_wrist_2, left_gamma, left_phi));

      // Clear and add the upper arm score
      upper.data.clear();
      upper.data.push_back(upper_arm_score);

      // Clear and add the lower arm score
      lower.data.clear();
      lower.data.push_back(lower_arm_score);

      // Clear and add the wrist score
      wrist_score_array.data.clear();
      wrist_score_array.data.push_back(wrist_score);

      // Clear and add the wrist twist score
      wrist_twist.data.clear();
      wrist_twist.data.push_back(wrist_twist_score);

      // Clear and add the abduction status
      abduct_sent.data.clear();
      abduct_sent.data.push_back(abduct_flag);

      // Clear and add the lateral status
      lateral_sent.data.clear();
      lateral_sent.data.push_back(lateral_flag);

      // Clear and add all the scores
      total_score.data.clear();
      total_score.data.push_back(rula(left_alpha, left_theta, left_wrist_1,left_wrist_2, left_gamma, left_phi));
      total_score.data.push_back(upper_arm_score);
      total_score.data.push_back(lower_arm_score);

      // Publish all the scores and status
      pub.publish(rula_score);
      pub_total.publish(total_score);
      pub_upper.publish(upper);
      pub_lower.publish(lower);
      pub_wrist.publish(wrist_score_array);
      pub_wrist_twist.publish(wrist_twist);
      abduct_pub.publish(abduct_sent);
      lateral_pub.publish(lateral_sent);

      // Spin and sleep for 1/40 s
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }
