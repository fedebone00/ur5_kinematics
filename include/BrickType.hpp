#pragma once

#include <string>
#include <eigen3/Eigen/Eigen>

class BrickType {
private:
  int x;
  int y;
  int z;
public:
  BrickType(int x, int y, int z, std::string name);
  std::string name;
  int getX() {return this->x;};
  int getY() {return this->y;};
  int getZ() {return this->z;};
  bool operator==(const BrickType &b);
};

class BrickTypeHash {
  public:
 
    // Use sum of lengths of first and last names
    // as hash function.
    size_t operator()(const BrickType& bt) const
    {
        return std::hash<std::string>()(bt.name);
    }
};