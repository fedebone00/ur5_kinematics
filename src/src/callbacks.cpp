#include "callbacks.hpp"
#include <iostream>

float _current_angle[6];
std::queue<ur5_gazebo::brick::ConstPtr> bricks;
int gotblocks = 0;


void shoulderPan_callback(const control_msgs::JointControllerState::ConstPtr &msg) {_current_angle[0] = msg->set_point;}
void shoulderLift_callback(const control_msgs::JointControllerState::ConstPtr &msg) {_current_angle[1] = msg->set_point;}
void elbow_callback(const control_msgs::JointControllerState::ConstPtr &msg) {_current_angle[2] = msg->set_point;}
void wrist1_callback(const control_msgs::JointControllerState::ConstPtr &msg) {_current_angle[3] = msg->set_point;}
void wrist2_callback(const control_msgs::JointControllerState::ConstPtr &msg) {_current_angle[4] = msg->set_point;}
void wrist3_callback(const control_msgs::JointControllerState::ConstPtr &msg) {_current_angle[5] = msg->set_point;}
void cisco_message_callback(const ur5_gazebo::brick::ConstPtr &brick) {bricks.push(brick);}
void gotblocks_message_callback(const std_msgs::Int8 &states){gotblocks = 1;}
