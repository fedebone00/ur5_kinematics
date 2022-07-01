// #include <eigen3/Eigen/src/Core/Matrix.h>
#include <ostream>
#include <robot.hpp>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "callbacks.hpp"
#include <eigen3/Eigen/Eigen>
#include "../include/ur5.hpp"
#include <cmath>

float reset_angle[] = {0, -1.24, 1.51, -1.84, -PI / 2, 0};
float behind_angle[] = {PI - 0.5, -1.24, 1.51, -1.84, -PI / 2, 0};

Robot::Robot(Eigen::Vector<double, 6> A, Eigen::Vector<double, 6> D, ros::NodeHandle &node): arm(A,D){
  this->pubs[0] = node.advertise<std_msgs::Float64>(
      "/shoulder_pan_joint_position_controller/command", RATE);
  this->pubs[1] = node.advertise<std_msgs::Float64>(
      "/shoulder_lift_joint_position_controller/command", RATE);
  this->pubs[2] = node.advertise<std_msgs::Float64>(
      "/elbow_joint_position_controller/command", RATE);
  this->pubs[3] = node.advertise<std_msgs::Float64>(
      "/wrist_1_joint_position_controller/command", RATE);
  this->pubs[4] = node.advertise<std_msgs::Float64>(
      "/wrist_2_joint_position_controller/command", RATE);
  this->pubs[5] = node.advertise<std_msgs::Float64>(
      "/wrist_3_joint_position_controller/command", RATE);

  this->subs[0] =
      node.subscribe("/shoulder_pan_joint_position_controller/state", RATE,
                     shoulderPan_callback);
  this->subs[1] =
      node.subscribe("/shoulder_lift_joint_position_controller/state", RATE,
                     shoulderLift_callback);
  this->subs[2] = node.subscribe("/elbow_joint_position_controller/state", RATE,
                                 elbow_callback);
  this->subs[3] = node.subscribe("/wrist_1_joint_position_controller/state",
                                 RATE, wrist1_callback);
  this->subs[4] = node.subscribe("/wrist_2_joint_position_controller/state",
                                 RATE, wrist2_callback);
  this->subs[5] = node.subscribe("/wrist_3_joint_position_controller/state",
                                 RATE, wrist3_callback);
/*
  this->LinkAtt = node.serviceClient<gazebo_ros_link_attacher::Attach>(
      "/link_attacher_node/attach");
  this->LinkDet = node.serviceClient<gazebo_ros_link_attacher::Attach>(
      "/link_attacher_node/detach");
      */

  this->grip_pub = node.advertise<std_msgs::Float64>(
      "/robotiq_85_left_knuckle_joint_position_controller/command", RATE);

  double curr_angles[6] = {0,-M_PI_2,M_PI_2,0,0,0};
  this->current = Eigen::Map<Eigen::Vector<double, 6>>(curr_angles);

    ros::Duration(1).sleep();
    while(!ros::ok());
    this->setAngle(curr_angles);
    ros::spinOnce();
    this->openGripper();
}

void Robot::closeGripper(void)/* ros::Rate loop_rate) */{
  std_msgs::Float64 msg;
  msg.data = 1;
  this->grip_pub.publish(msg);
  ros::spinOnce();
    ros::Duration(1.5).sleep();
  // loop_rate.sleep();
}

void Robot::openGripper(void)/* ros::Rate loop_rate) */{
  std_msgs::Float64 msg;
  msg.data = 0.1;
  this->grip_pub.publish(msg);
  ros::spinOnce();
    ros::Duration(1.5).sleep();
  // loop_rate.sleep();
}

void Robot::setAngle(double angle[6]){
  std_msgs::Float64 tmp;
  for(int i = 0; i < 6; i++) 
  {
    tmp.data = angle[i];
    this->pubs[i].publish(tmp);
    ros::Duration(0.005).sleep();
  }
  ros::spinOnce();
}

bool Robot::goTo(pos_rot_TypeDef pos_rot) {
  Eigen::Matrix<double, 8, 6> qET = this->arm.IK(pos_rot.pe, pos_rot.Re);

  int i = 0;
  while(i < qET.size() && (qET.row(i).hasNaN() || (qET.row(i).array() > 4).any() || (qET.row(i).array() < -4).any())) ++i;

  if(!(i < qET.size())) return false;

  Eigen::Vector<double, 6> qE = qET.row(i);

  return this->goTo(qE);
}

bool Robot::goTo(Eigen::Vector<double, 6> Th) {
  std_msgs::Float64 tmp;
  if(Th.hasNaN()) return false;
  for(int i = 0; i < 6; ++i) 
  {
    tmp.data = Th[i];
    this->pubs[i].publish(tmp);
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
  this->setCurrent(Th);
  return true;
}

bool Robot::goFromCurrentToPos(pos_rot_TypeDef pos, int steps) {
  Eigen::Vector<double, 6> pos_v = this->arm.get_best_vector(this->current, pos);

  if(pos_v == Eigen::Vector<double, 6>::Zero()) return false;

  auto traj = this->arm.P2P(this->current,pos_v, 0, steps, 1);
  if(traj.size() == 0) {std::cout << "leaving" << std::endl; return false;}

  //arriva sopra al blocco
  for(auto it = traj.begin(); it < traj.end(); ++it) {
    if(!this->goTo(it->Th)) return false;
    ros::Duration(0.005).sleep();
  }

  return true;
}

bool Robot::moveBlock(pos_rot_TypeDef start, pos_rot_TypeDef end) {
  pos_rot_TypeDef upstart = start, upend = end;

  upstart.pe = upstart.pe.array() + Eigen::Array3d(0, 0, 0.2);
  upend.pe = upend.pe.array() + Eigen::Array3d(0, 0, 0.2);

  if(!(this->goFromCurrentToPos(upstart, 40))) return false;
  std::cout << "upstart done\n";
  if(!(this->goFromCurrentToPos(start, 15))) return false;
  std::cout << "start done\n";
  this->closeGripper();
  if(!(this->goFromCurrentToPos(upstart, 15))) return false;
  std::cout << "upstart2 done\n";
  if(!(this->goFromCurrentToPos(upend, 40))) return false;
  std::cout << "upend done\n";
  if(!(this->goFromCurrentToPos(end, 15))) return false;
  std::cout << "end done\n";
  this->openGripper();
  if(!(this->goFromCurrentToPos(upend, 15))) return false;
  std::cout << "upend2 done\n";
  return true;
}

void Robot::setCurrent(Eigen::Vector<double, 6> c) {
  this->current = c;
}
