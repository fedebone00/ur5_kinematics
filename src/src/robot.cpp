// #include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
#include <robot.hpp>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "std_msgs/Float64.h"
#include "callbacks.hpp"
#include <eigen3/Eigen/Eigen>
#include "../include/ur5.hpp" //cambiato a caso [tolto ""]
#include <cmath>
#include <string>
#include "BrickType.hpp"
#include "Brick.hpp"
#include "ros_utils.hpp"

extern EndPoint la_terra_di_mezzo;


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

  this->grip_pub = node.advertise<std_msgs::Float64>(
      "/robotiq_85_left_knuckle_joint_position_controller/command", RATE);

  double curr_angles[6] = {0,-M_PI_2,M_PI_2,0,0,0};
  this->setCurrent(Eigen::Map<Eigen::Vector<double, 6>>(curr_angles));

    ros::Duration(1).sleep();
    while(!ros::ok());
    this->setAngle(curr_angles);
    ros::spinOnce();
    this->openGripper("");
}

bool Robot::linker(ros::ServiceClient node, std::string model, std::string link) {
  if(!(ros_link(node, LINK_ATTACHER_UR5_MODEL, LINK_ATTACHER_UR5_LINK1, model, link))) return false;

  return ros_link(node, LINK_ATTACHER_UR5_MODEL, LINK_ATTACHER_UR5_LINK2, model, link);
}

void Robot::closeGripper(std::string blockName,int blockDim)/* ros::Rate loop_rate) */{
  std_msgs::Float64 msg;
  msg.data = 1 - (blockDim * 0.041/0.085);
  this->grip_pub.publish(msg);
  ros::spinOnce();
  ros::Duration(1.5).sleep();
  if (blockName.length() > 0) this->linker(LinkAtt, blockName, "link");
  // loop_rate.sleep();
}

void Robot::openGripper(std::string blockName)/* ros::Rate loop_rate) */{
  std_msgs::Float64 msg;
  msg.data = 0.;
  this->grip_pub.publish(msg);
  if (blockName.length() > 0) this->linker(LinkDet, blockName, "link");
  ros::spinOnce();
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

bool Robot::moveBlock(pos_rot_TypeDef start, pos_rot_TypeDef end, std::string lastModel, std::string lastLink, std::string blockName, std::string blockLink, int blockDim) {
  pos_rot_TypeDef upstart = start, upend = end;

  upstart.pe = upstart.pe.array() + Eigen::Array3d(0, 0, 0.2);
  upend.pe = upend.pe.array() + Eigen::Array3d(0, 0, 0.2);

  if(!(this->goFromCurrentToPos(upstart, 40))) return false;
  std::cout << "upstart done\n";
  if(!(this->goFromCurrentToPos(start, 15))) return false;
  std::cout << "start done\n";
  this->closeGripper(blockName, blockDim);
  //if(lastModel != "") ros_link(LinkDet, lastModel, lastLink, blockName, blockLink);
  if(!(this->goFromCurrentToPos(upstart, 15))) return false;
  std::cout << "upstart2 done\n";
  if(!(this->goFromCurrentToPos(upend, 40))) return false;
  std::cout << "upend done\n";
  if(!(this->goFromCurrentToPos(end, 40))) return false;
  std::cout << "end done\n";
  if(lastModel != "") {if(!ros_link(LinkAtt, lastModel, lastLink, blockName, blockLink)) return false;}
  this->openGripper(blockName);
  ros::Duration(0.5).sleep();
  if(!(this->goFromCurrentToPos(upend, 15))) return false;
  std::cout << "upend2 done\n";
  return true;
}

bool Robot::moveBlockToEndpoint(Brick &brick, EndPoint &end) {

  pos_rot_TypeDef pos = brick.pose();
  pos_rot_TypeDef end_pos = end.getPose();
  //find parallel axis

  Eigen::Vector3d newUnitZ = pos.Re*Eigen::Vector3d::UnitZ();

  newUnitZ = newUnitZ.array() * Eigen::Array3d(-1, -1, 1);

  double scalarX = (pos.Re*Eigen::Vector3d::UnitX()).dot(Eigen::Vector3d::UnitZ());
  double scalarY = (pos.Re*Eigen::Vector3d::UnitY()).dot(Eigen::Vector3d::UnitZ());
  double scalarZ = (newUnitZ).dot(Eigen::Vector3d::UnitZ());

  double max = std::max(std::max(std::abs(scalarX), std::abs(scalarY)), std::abs(scalarZ));

  int mainDim = 0;
  double r, p, y = 0;

  if(max == std::abs(scalarX)) { //non raggiunge
    r = M_PI_2 + std::signbit(scalarX) ? -M_PI_2 : M_PI_2;
    p = 0;
    y = M_PI;
    mainDim = brick.type()->getY();
  } else if(max == std::abs(scalarY)) {
    y = M_PI_2;
    pos.pe += newUnitZ * GRIPPER_LENGTH;
    pos.pe(2) += 0.009;
    /*
    r = M_PI_2 - (std::signbit(scalarY) ? -M_PI_2 : M_PI_2);
    std::cout << "r = " << r << std::endl << "scalarY = " << scalarY << std::endl;
    p = 0;
    y = M_PI_2 - (std::signbit(scalarY) ? -M_PI_2 : M_PI_2);
    */
    mainDim = brick.type()->getX();
  } else if(max == scalarZ) {
    r = 0;
    p = M_PI;
    y = brick.type()->getX() > brick.type()->getY() ? M_PI_2 : 0;
    pos.pe(2) += GRIPPER_LENGTH;
    mainDim = std::min(brick.type()->getX(), brick.type()->getY());
  } else if(max == -scalarZ) {
    r = 0;
    p = 0;
    y = brick.type()->getX() > brick.type()->getY() ? M_PI_2 : 0;
    pos.pe(2) += GRIPPER_LENGTH;
    mainDim = std::min(brick.type()->getX(), brick.type()->getY());

    std::string brickname = brick.name() + "-" + std::to_string(state);
    std::cout << brickname << std::endl;
    pos.Re = eul2rotm(Eigen::Vector3d(r, p, y))*pos.Re;
    if(moveBlock(pos, la_terra_di_mezzo.getPose(), la_terra_di_mezzo.lastBlockModel(), la_terra_di_mezzo.lastBlockLink(), brickname, "link", mainDim)) {
      std::cout << "second movement" << std::endl;
      pos_rot_TypeDef p = la_terra_di_mezzo.getPose();
      p.pe(0) -= GRIPPER_LENGTH;
      p.pe(2) = 0.0189;
      p.Re = eul2rotm(Eigen::Vector3d(0,-M_PI_2,0)) * p.Re;

      pos_rot_TypeDef p2 = p;
      p2.pe(2) += 0.004;
      p2.Re = eul2rotm(Eigen::Vector3d(0,M_PI_2,M_PI)) * la_terra_di_mezzo.getPose().Re;



      if(moveBlock(p, p2, la_terra_di_mezzo.lastBlockModel(), la_terra_di_mezzo.lastBlockLink(), brickname, "link", mainDim)) {
        std::cout << "third movement" << std::endl;

        p = la_terra_di_mezzo.getPose();
        p.pe(2) -= 0.005;

        end_pos.pe(2) += 0.004;

        if(moveBlock(p, end_pos, end.lastBlockModel(), end.lastBlockLink(), brickname, "link", mainDim)) {
          end.changeLastBlock(brickname, "link", brick.type()->getZ());
          return true;
        } 
        else return false;
      }
      //if(moveBlock(p, )) return true;
    }



  }
  std::string brickname = brick.name() + "-" + std::to_string(state);
  std::cout << brickname << std::endl;
  pos.Re = eul2rotm(Eigen::Vector3d(r, p, y))*pos.Re;
  if(moveBlock(pos, end.getPose(), end.lastBlockModel(), end.lastBlockLink(), brickname, "link", mainDim)) {
    end.changeLastBlock(brickname, "link", brick.type()->getZ());
    return true;
  }









/*


  if(max == std::abs(scalarX)) {
    pos.Re = eul2rotm(Eigen::Vector3d(0, M_PI, 0))*pos.Re;
    pos.pe += Eigen::Vector3d()
    if(moveBlock(pos, end.getPose(), brick.name(), brick.type()->getY())) {
      if(!ros_link(LinkAtt, end.lastBlockModel(), end.lastBlockLink(), brick.name(), "link")) return false;
      end.changeLastBlock(brick.name(), "link", brick.type()->getZ());
      return true;
    }
  } else if(max == std::abs(scalarY)) {
    pos.Re = eul2rotm(Eigen::Vector3d(0, M_PI, M_PI_2))*pos.Re;
    if(moveBlock(pos, end.getPose(), brick.name(), brick.type()->getX())) {
      if(!ros_link(LinkAtt, end.lastBlockModel(), end.lastBlockLink(), brick.name(), "link")) return false;
      end.changeLastBlock(brick.name(), "link", brick.type()->getZ());
      return true;
    }

  } else if(max == scalarZ) { //il blocco è dritto e non va girato
    pos.Re = eul2rotm(Eigen::Vector3d(0, M_PI, brick.type()->getX() > brick.type()->getY() ? M_PI_2 : 0))*pos.Re;
    if(moveBlock(pos, end.getPose(), brick.name(), std::min(brick.type()->getX(), brick.type()->getY()))) {
      if(!ros_link(LinkAtt, end.lastBlockModel(), end.lastBlockLink(), brick.name(), "link")) return false;
      end.changeLastBlock(brick.name(), "link", brick.type()->getZ());
      return true;
    }
  } else {

  }
  */
/*
  double approx_z = (long)((rotated(2)+0.5)*10) / 10;
  if(approx_z == Eigen::Vector3d::UnitZ()(2)) { //il blocco è dritto e non va girato
    pos.Re =  eul2rotm(brick.type()->getRPY())*pos.Re;
    if(moveBlock(pos, end.getPose(), brick.name(), brick.type()->getMainDim())) {
      if(!ros_link(LinkAtt, end.lastBlockModel(), end.lastBlockLink(), brick.name(), "link")) return false;
      end.changeLastBlock(brick.name(), "link", brick.type()->getZ());
      return true;
    }
  }
*/

/*

  if(moveBlock(brick.pose(), end.getPose(), brick.name())) {
    std::cout << "linking " << end.lastBlockModel() << " : " << end.lastBlockLink() << " to " << brick.name();
    if(!ros_link(LinkAtt, end.lastBlockModel(), end.lastBlockLink(), brick.name(), "link")) return false;
    end.changeLastBlock(brick.name(), "link", brick.type()->getZ());
    return true;
  }
  return false;
  */
 return false;
}

void Robot::setCurrent(Eigen::Vector<double, 6> c) {
  this->current = c;
}


Eigen::Vector<double, 6> Robot::getCurrent(void) {
  return this->current;
}
