#include "EndPoint.hpp"
#include "robot.hpp"

#define EDGE_BASE 0.0195

EndPoint::EndPoint() {};
EndPoint::EndPoint(Eigen::Vector3d pe, std::string lastBlockModel, std::string lastBlockLink) {
      this->pose.pe = Eigen::Vector3d(-pe(0), -pe(1), pe(2) - 0.88 + GRIPPER_LENGTH);
      this->pose.Re = eul2rotm(Eigen::Vector3d(0,M_PI, M_PI_2));
      this->_lastBlockModel = lastBlockModel;
      this->_lastBlockLink = lastBlockLink;
      this->number = 0;
}

pos_rot_TypeDef EndPoint::getPose(void) {
  pos_rot_TypeDef tmp = this->pose;
  tmp.pe += Eigen::Vector3d(0,0,this->number*EDGE_BASE);
  return tmp;
}

void EndPoint::changeLastBlock(std::string newBlockModel, std::string newBlockLink, int newHeight){
  this->_lastBlockModel = newBlockModel;
  this->_lastBlockLink = newBlockLink;
  this->number += newHeight;
}

CastlePoint::CastlePoint(Eigen::Vector3d pe, BrickType type, std::string lastBlockModel, std::string lastBlockLink) : EndPoint(pe, lastBlockModel, lastBlockLink), type(type) {}

BrickType CastlePoint::getType(){
  return this->type;
}
