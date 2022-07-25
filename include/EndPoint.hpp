#pragma once

#include "ur5.hpp"
#include <string>

#include "BrickType.hpp"

 // Class EndPoint: used to define the point where the block will
 //                 be paced (placed above if already there)

class EndPoint
{
public:
  EndPoint();
  EndPoint(Eigen::Vector3d pe, std::string lastBlockModel = "", std::string lastBlockLink = "");
  EndPoint(EndPoint &&) = default;
  EndPoint(const EndPoint &) = default;
  EndPoint &operator=(EndPoint &&) = default;
  EndPoint &operator=(const EndPoint &) = default;
  ~EndPoint() = default;
  pos_rot_TypeDef getPose(void);
  void changeLastBlock(std::string newBlockModel, std::string newBlockLink, int newHeight);
  std::string lastBlockModel() {return this->_lastBlockModel;};
  std::string lastBlockLink() {return this->_lastBlockLink;};

protected:
  pos_rot_TypeDef pose;
  int number;
  std::string _lastBlockModel;
  std::string _lastBlockLink;
};

class CastlePoint : public EndPoint
{
public:
  CastlePoint(Eigen::Vector3d pe, BrickType type, std::string lastBlockModel = "", std::string lastBlockLink = "");
  CastlePoint(CastlePoint &&) = default;
  CastlePoint(const CastlePoint &) = default;
  CastlePoint &operator=(CastlePoint &&) = default;
  CastlePoint &operator=(const CastlePoint &) = default;
  ~CastlePoint() = default;
  BrickType getType();
private:
  BrickType type;
};

