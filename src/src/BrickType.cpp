#include "BrickType.hpp"


BrickType::BrickType(int x, int y, int z, std::string name) : x(x), y(y), z(z), name(name) { /*sono un costrutture stupido*/ }

bool BrickType::operator==(const BrickType &b) {
  return this->name == b.name && this->x == b.x && this->y == b.y && this->z == b.z;
}