#include "Brick.hpp"

#include <iostream>

Brick::Brick(pos_rot_TypeDef pose, BrickType* type) {
    int zone = 0;
    zone += pose.pe(0) < -0.55/*limite centrale x*/ ? 2 : 0;
    zone += pose.pe(1) < 0/*limite centrale y*/ ? 1 : 0;

    this->_pose = pose;
    this->_type = type;
    this->_name = type->name + "-" + std::to_string(zone);

    std::cout << this->_name << std::endl;
}