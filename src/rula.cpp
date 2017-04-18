#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <math.h>
// #include <random>

using namespace std;

double freq = 40.0;
double dt = 1.0/freq;
double left_a, left_b, left_c, left_d, left_e, left_f;
double right_a, right_b, right_c, right_d;
int row,col;
int step1, step2, step3, step4;
int abduct_flag, lateral_flag;

void get_angle_left(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64MultiArray pathv;
  pathv = *msg;
  left_b  = pathv.data[0];
  left_a = pathv.data[1];
  left_c = pathv.data[2];
  left_d = pathv.data[3];
  left_e = pathv.data[4];
  left_f = pathv.data[5];
}

void get_angle_right(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64MultiArray pathv;
  pathv = *msg;
  right_b  = pathv.data[0];
  right_a = pathv.data[1];
  right_c = pathv.data[2];
  right_d = pathv.data[3];
}

int rulatable[144] = {1,2,2,2,2,3,3,3,2,2,2,2,3,3,3,3,2,3,3,3,3,3,4,4,2,3,3,3,3,4,4,4,3,3,3,3,3,4,4,4,3,4,4,4,4,4,5,5,3,3,4,4,4,4,5,5,3,4,4,4,4,4,5,5,4,4,4,4,4,5,5,5,4,4,4,4,4,5,5,5,4,4,4,4,4,5,5,5,4,4,4,5,5,5,6,6,5,5,5,5,5,6,6,7,5,6,6,6,6,7,7,7,6,6,6,7,7,7,7,8,7,7,7,7,7,8,8,9,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,9};

int rula(double a, double b, double c, double d, double e, double f)
{
int index, ergo;


//Step 1:
if (a>=-10 && a<20) {step1=1;}
else if (a<-10){step1=2;}
else if (a>=20 && a<45){step1=2;}
else if (a>=45 && a<90) step1=3;
else if (a>=90){step1=4;}
else {step1=0;}

//Abduction
if(e>25)
{
  step1 = step1 + 1;
  abduct_flag = 1;
}
else abduct_flag = 0;

//Step 2:
  if (b>=50 && b<100) {step2=1;}
  if (b<50 || b>=100) {step2=2;}
  if(f>25 || f<-25)
  {
    step2 = step2 + 1;
    lateral_flag = 1;
  }
  else lateral_flag = 0;

//Step 3:
  if ((c>=-40 && c<-10) || (c<40 && c>=10)) step3=1;
  else if (c>=40 || c<-40) step3=2;
  else if(c>=-10 && c<10) step3=1;

  // if(d<-10 || d>10) step3 = step3+1;


  //Trial:
  //step3 = 2; step4 = 1;
  step4 = 1;

//Step 5:
  col = (step3)*2 - (2-step4);
  row = step1*3 - (3-step2);
  index = (row-1)*8 + col-1;
  ergo = rulatable[index];
  return ergo;
}

int
main(int argc, char** argv)
  {
    ros::init(argc,argv, "rula");
    ros::NodeHandle n;
    abduct_flag = 0;
    ros::Rate loop_rate(freq);
    std_msgs::Float64MultiArray rula_score, rula_coord, upper, lower, wrist_score, wrist_twist, total_score;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("rula_score",10);
    ros::Publisher pub_total = n.advertise<std_msgs::Float64MultiArray>("total_score",10);
    ros::Publisher pub_upper = n.advertise<std_msgs::Float64MultiArray>("upper_arm_score",10);
    ros::Publisher pub_lower = n.advertise<std_msgs::Float64MultiArray>("lower_arm_score",10);
    ros::Publisher pub_wrist = n.advertise<std_msgs::Float64MultiArray>("wrist_score",10);
    ros::Publisher pub_wrist_twist = n.advertise<std_msgs::Float64MultiArray>("wrist_twist_score",10);
    ros::Publisher coord_pub = n.advertise<std_msgs::Float64MultiArray>("rula_coordinate",10);
    ros::Publisher abduct_pub = n.advertise<std_msgs::Int32MultiArray>("abduction_status",10);
    ros::Publisher lateral_pub = n.advertise<std_msgs::Int32MultiArray>("lateral_status",10);
    ros::Subscriber left_angle = n.subscribe("left_human_angle", 10, get_angle_left);
    ros::Subscriber right_angle = n.subscribe("right_human_angle", 10, get_angle_right);
    std_msgs::Int32MultiArray abduct_sent;
    std_msgs::Int32MultiArray lateral_sent;
    while(ros::ok())
    {
      rula_score.data.clear();
      rula_score.data.push_back(rula(left_a, left_b, left_c,left_d, left_e, left_f));

      rula_coord.data.clear();
      rula_coord.data.push_back(row);
      rula_coord.data.push_back(col);

      upper.data.clear();
      upper.data.push_back(step1);

      lower.data.clear();
      lower.data.push_back(step2);

      wrist_score.data.clear();
      wrist_score.data.push_back(step3);

      wrist_twist.data.clear();
      wrist_twist.data.push_back(step4);

      abduct_sent.data.clear();
      abduct_sent.data.push_back(abduct_flag);

      lateral_sent.data.clear();
      lateral_sent.data.push_back(lateral_flag);

      total_score.data.clear();
      total_score.data.push_back(rula(left_a, left_b, left_c,left_d, left_e, left_f));
      total_score.data.push_back(step1);
      total_score.data.push_back(step2);
      // cout << left_c << endl;
//       rula_score.data.push_back(rula(right_a, right_b, right_c));
//       rula_score.data.push_back(rula(30.0, 60.0, 0.0));
      pub.publish(rula_score);
      pub_total.publish(total_score);
      coord_pub.publish(rula_coord);
      pub_upper.publish(upper);
      pub_lower.publish(lower);
      pub_wrist.publish(wrist_score);
      pub_wrist_twist.publish(wrist_twist);
      abduct_pub.publish(abduct_sent);
      lateral_pub.publish(lateral_sent);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }
