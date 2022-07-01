#pragma once

#include "../include/ur5.hpp"
#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>

// global variables

#define PI 3.14159265358979323846
#define RATE 100

extern float reset_angle[];
extern float behind_angle[];

// pos_rot_TypeDef final_positions[] = {{}
// 			    };

class Robot
{
public:
  Robot(Eigen::Vector<double, 6> A, Eigen::Vector<double, 6> D, ros::NodeHandle &node);
  ~Robot() = default;
  bool moveBlock(pos_rot_TypeDef starting, pos_rot_TypeDef final);
  void setAngle(double angles[6]);
  void setCurrent(Eigen::Vector<double, 6> c);
  bool goTo(pos_rot_TypeDef pos_rot);
  bool goTo(Eigen::Vector<double, 6> Th);
private:
  UR5 arm;
  Eigen::Vector<double, 6> current;
  ros::Publisher pubs[6];
  ros::Publisher grip_pub;
  ros::Subscriber subs[6];
  // ros::ServiceClient LinkAtt;
  // ros::ServiceClient LinkDet;

  void closeGripper(/*ros::Rate rate*/);
  void openGripper(/*ros::Rate rate*/);
  void updateCurrent();
  bool goFromCurrentToPos(pos_rot_TypeDef pos, int maxT);
};
