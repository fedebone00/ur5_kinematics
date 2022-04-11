#include <eigen3/Eigen/Eigen>

typedef struct {
    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
} pos_rot;

typedef struct {
    uint t;
    Eigen::Vector<double, 6> Th;
    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
} t_th_pos_rot;

typedef Eigen::VectorX<t_th_pos_rot> Trajectory;

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
public:
    UR5(Eigen::Vector<double, 6> A, Eigen::Vector<double, 6> D);
    Eigen::Matrix3d eul2rotm(Eigen::Vector3d rpy);
    pos_rot DK(Eigen::Vector<double, 6> Th);
    Eigen::Matrix<double, 8, 6> IK(Eigen::Vector3d p60, Eigen::Matrix3d R60);
    Trajectory P2P(pos_rot starting, pos_rot final, uint minT, uint maxT, uint dt);
};