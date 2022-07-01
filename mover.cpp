#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
//#include "std_msgs/JointStateController.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>
// sudo apt install libeigen3-dev
#include <eigen3/Eigen/Eigen>
// srv from gazebo_ros_link_attacher
#include "gazebo_ros_link_attacher/Attach.h"
#include <cmath>

using namespace std;

#include "robot.hpp"
#include "callbacks.hpp"
#include "ur5.hpp"

Eigen::Vector<double, 6> A(0, -0.425, -0.3922, 0, 0, 0);
Eigen::Vector<double, 6> D(0.1625, 0, 0, 0.1333, 0.0997, 0.0996);

int main(int argc, char **argv) {
  ros::init(argc, argv, "mover");
  ros::NodeHandle node;
  ros::Rate loop_rate(RATE);
  
  Robot arm(A, D, node);

  ros::Subscriber cisco = node.subscribe("/motion/plan", RATE, cisco_message_callback);
  //ros::Subscriber states = node.subscribe("/gazebo/link_states", RATE, states_message_callback);
  
  //arm.setAngle(behind_angle);
  /* ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep(); */

  time_t start = std::time(0);

  while(ros::ok()){
    ros::spinOnce();
    if(!bricks.empty()) {
      Eigen::Vector3d pe;
      Eigen::Vector3d rpy;
      kinematics::brick::ConstPtr brick = bricks.front();
      bricks.pop();
      int i=0;
      for(auto x : brick->initPosition) {
        pe(i++) = x;
      }
      i=0;
      for(auto x : brick->initRotation) {
        rpy(i++) = x;
      }
      pos_rot_TypeDef brick_pos = {
        .pe = pe,
        .Re = eul2rotm(rpy)
      };
      pos_rot_TypeDef final = {
        // .pe = Eigen::Vector3d(0.494228,0.3,0.2),
        .pe = Eigen::Vector3d(-0.5,-0.5,0.2),
        .Re = eul2rotm(Eigen::Vector3d(M_PI, M_PI, 0))
      };

      if(!(arm.moveBlock(brick_pos, final))) std::cout << "failed" << std::endl;
    }
  }
  time_t end = std::time(0);
  cout << "Time of execution: " << end - start;

  return 0;
}
