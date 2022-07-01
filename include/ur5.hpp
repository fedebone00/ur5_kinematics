#pragma once

#include <eigen3/Eigen/Eigen>

typedef struct pr {
  Eigen::Vector3d pe;
  Eigen::Matrix3d Re;
  struct pr &operator=(const struct pr &);
} pos_rot_TypeDef;

typedef struct {
  float t;
  Eigen::Vector<double, 6> Th;
} t_th_TypeDef;

typedef struct {
  pos_rot_TypeDef position;
  int type;
} block;

typedef Eigen::VectorX<t_th_TypeDef> Trajectory;

class UR5 {
private:
  Eigen::Vector<double, 6> A;
  Eigen::Vector<double, 6> D;
  Eigen::Matrix4d T10f(double th1);
  Eigen::Matrix4d T21f(double th2);
  Eigen::Matrix4d T32f(double th3);
  Eigen::Matrix4d T43f(double th4);
  Eigen::Matrix4d T54f(double th5);
  Eigen::Matrix4d T65f(double th6);
  const Eigen::Vector<double, 6> best_angles(const Eigen::Vector<double, 6> &actual, const Eigen::Matrix<double, 8, 6> &possible);
  static double normalize(double);
public:
  UR5(Eigen::Vector<double, 6> A, Eigen::Vector<double, 6> D);
  Eigen::Vector<double, 6> get_best_vector(const Eigen::Vector<double, 6> &actual, const pos_rot_TypeDef &final);
  pos_rot_TypeDef DK(Eigen::Vector<double, 6> Th);
  Eigen::Matrix<double, 8, 6> IK(Eigen::Vector3d p60, Eigen::Matrix3d R60);
  Trajectory P2P(Eigen::Vector<double, 6> starting, Eigen::Vector<double, 6> final, uint minT, uint maxT, uint dt);
  friend Eigen::Matrix3d eul2rotm(Eigen::Vector3d rpy);
};

Eigen::Matrix3d eul2rotm(Eigen::Vector3d rpy);
