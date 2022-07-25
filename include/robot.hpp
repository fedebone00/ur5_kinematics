#pragma once

#include "../include/ur5.hpp"
#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <queue>
#include "EndPoint.hpp"
#include "Brick.hpp"

// global variables

#define RATE 10
#define LINK_ATTACHER_UR5_MODEL "robot"
#define LINK_ATTACHER_UR5_LINK1 "robotiq_85_left_finger_tip_link"
#define LINK_ATTACHER_UR5_LINK2 "robotiq_85_right_finger_tip_link"
#define GRIPPER_LENGTH 0.171

extern int state;
class Robot
{
public:
  Robot(Eigen::Vector<double, 6> A, Eigen::Vector<double, 6> D, ros::NodeHandle &node);
  ~Robot() = default;
  bool moveBlock(pos_rot_TypeDef start, pos_rot_TypeDef end, std::string lastModel, std::string lastLink, std::string blockName, std::string blockLink, int blockDim);
  bool moveBlockk(pos_rot_TypeDef start, pos_rot_TypeDef end, std::string lastModel, std::string lastLink, std::string blockName, std::string blockLink, int blockDim);
  bool moveBlockToEndpoint(Brick &brick, EndPoint &end);
  void setAngle(double angles[6]);
  void setCurrent(Eigen::Vector<double, 6> c);
  Eigen::Vector<double, 6> getCurrent(void);
  bool goTo(pos_rot_TypeDef pos_rot);
  bool goTo(Eigen::Vector<double, 6> Th);
private:
  UR5 arm;
  Eigen::Vector<double, 6> current;
  ros::Publisher pubs[6];
  ros::Publisher grip_pub;
  ros::Subscriber subs[6];

  void closeGripper(std::string blockName, int blockDim);
  void openGripper(std::string blockName);
  bool linker(ros::ServiceClient node, std::string model, std::string link);
  void updateCurrent();
  bool goFromCurrentToPos(pos_rot_TypeDef pos, int maxT);
};
