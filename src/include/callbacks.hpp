#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "kinematics/brick.h"
#include "gazebo_msgs/LinkStates.h"
#include <queue>
#include <eigen3/Eigen/Eigen>

extern float _current_angle[6];
extern std::queue<kinematics::brick::ConstPtr> bricks;

void shoulderPan_callback(const control_msgs::JointControllerState::ConstPtr &msg);
void shoulderLift_callback(const control_msgs::JointControllerState::ConstPtr &msg);
void elbow_callback(const control_msgs::JointControllerState::ConstPtr &msg);
void wrist1_callback(const control_msgs::JointControllerState::ConstPtr &msg);
void wrist2_callback(const control_msgs::JointControllerState::ConstPtr &msg);
void wrist3_callback(const control_msgs::JointControllerState::ConstPtr &msg);
void cisco_message_callback(const kinematics::brick::ConstPtr &brick);
void states_message_callback(const gazebo_msgs::LinkStates::ConstPtr &states);
#endif
