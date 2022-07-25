#pragma once

#include "ur5.hpp"
#include "BrickType.hpp"

class Brick {
  private:
    std::string _name;
    pos_rot_TypeDef _pose;
    BrickType* _type;
  public:
    Brick(pos_rot_TypeDef pose, BrickType* type);
    ~Brick()=default;
    std::string name(){return this->_name;};
    pos_rot_TypeDef pose() {return this->_pose;};
    void setPose(pos_rot_TypeDef p) { this->_pose = p;};
    BrickType* type() {return this->_type;};
};