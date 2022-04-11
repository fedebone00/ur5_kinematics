#include "ur5.hpp"
#include <iostream>

UR5::UR5(Eigen::Vector<double, 6> A, Eigen::Vector<double, 6> D) : A(A), D(D) {/*ciao sono il costruttore*/}

Eigen::Matrix4d UR5::T10f(double th1) {
    Eigen::Matrix4d tmp;
    tmp << 
        cos(th1), -sin(th1), 0, 0,
        sin(th1), cos(th1), 0, 0,
        0, 0, 1, D(0),
        0, 0, 0, 1;
    return tmp;
}

Eigen::Matrix4d UR5::T21f(double th2) {
    Eigen::Matrix4d tmp;
    tmp << 
        cos(th2), -sin(th2), 0, 0,
        0, 0, -1, 0,
        sin(th2), cos(th2), 0, 0,
        0, 0, 0, 1;
    return tmp;
}

Eigen::Matrix4d UR5::T32f(double th3) {
    Eigen::Matrix4d tmp;
    tmp << 
        cos(th3), -sin(th3), 0, A(1),
        sin(th3), cos(th3), 0, 0,
        0, 0, 1, D(2),
        0, 0, 0, 1;
    return tmp;
}

Eigen::Matrix4d UR5::T43f(double th4) {
    Eigen::Matrix4d tmp;
    tmp << 
        cos(th4), -sin(th4), 0, A(2),
        sin(th4), cos(th4), 0, 0,
        0, 0, 1, D(3),
        0, 0, 0, 1;
    return tmp;
}

Eigen::Matrix4d UR5::T54f(double th5) {
    Eigen::Matrix4d tmp;
    tmp << 
        cos(th5), -sin(th5), 0, 0,
        0, 0, -1, -D(4),
        sin(th5), cos(th5), 0, 0,
        0, 0, 0, 1;
    return tmp;
}

Eigen::Matrix4d UR5::T65f(double th6) {
    Eigen::Matrix4d tmp;
    tmp << 
        cos(th6), -sin(th6), 0, 0,
        0, 0, 1, D(5),
        -sin(th6), -cos(th6), 0, 0,
        0, 0, 0, 1;
    return tmp;
}

Eigen::Matrix3d UR5::eul2rotm(Eigen::Vector3d rpy) {
    double a = rpy(0);
    double b = rpy(1);
    double g = rpy(2);
    Eigen::Matrix3d tmp;
    tmp <<
        cos(a)*cos(b),  cos(a)*sin(b)*sin(g)-sin(a)*cos(g), cos(a)*sin(b)*cos(g)+sin(a)*sin(g),
        sin(a)*cos(b),  sin(a)*sin(b)*sin(g)+cos(a)*cos(g), sin(a)*sin(b)*cos(g)-cos(a)*sin(g),
        -sin(b),        cos(b)*sin(g),                      cos(b)*cos(g);
    return tmp;
}

pos_rot UR5::DK(Eigen::Vector<double, 6> Th) {
    Eigen::Matrix4d T10m = T10f(Th(0));
    Eigen::Matrix4d T21m = T21f(Th(1));
    Eigen::Matrix4d T32m = T32f(Th(2));
    Eigen::Matrix4d T43m = T43f(Th(3));
    Eigen::Matrix4d T54m = T54f(Th(4));
    Eigen::Matrix4d T65m = T65f(Th(5));

    Eigen::Matrix4d T06 = T10m * T21m * T32m * T43m * T54m * T65m;
    pos_rot pr;
    pr.pe = T06.block(0, 3, 3, 1);
    pr.Re = T06.block(0, 0, 3, 3);
    return pr;
}

Eigen::Matrix<double, 8, 6> UR5::IK(Eigen::Vector3d p60, Eigen::Matrix3d R60) {
    Eigen::Matrix4d T60;
    
    T60 << R60, p60, 0, 0, 0, 1;

    Eigen::Vector4d p50 = T60 * Eigen::Vector4d(0, 0, -D(5), 1);
    double th1_1 = atan2(p50(1), p50(0)) + acos(D(3)/hypot(p50(1), p50(0))) + M_PI_2;
    double th1_2 = atan2(p50(1), p50(0)) - acos(D(3)/hypot(p50(1), p50(0))) + M_PI_2;
    
    double th5_1 = acos((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D(3)) / D(5));
    double th5_2 = -th5_1;
    double th5_3 = acos((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D(3)) / D(5));
    double th5_4 = -th5_3;

    Eigen::Matrix4d T06 = T60.inverse();
    Eigen::Vector3d Xhat = T06.block(0, 0, 3, 1);
    Eigen::Vector3d Yhat = T06.block(0, 1, 3, 1);

    double th6_1 = atan2((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))/sin(th5_1), (Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_1));
    double th6_2 = atan2((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1))/sin(th5_2), (Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1))/sin(th5_2));
    double th6_3 = atan2((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_3), (Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_3));
    double th6_4 = atan2((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2))/sin(th5_4), (Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2))/sin(th5_4));

    Eigen::Matrix4d T41m = T10f(th1_1).inverse() * T60 * T65f(th6_1).inverse() * T54f(th5_1).inverse();
    Eigen::Vector3d p41_1 = T41m.block(0, 3, 3, 1);
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    T41m = T10f(th1_1).inverse() * T60 * T65f(th6_2).inverse() * T54f(th5_2).inverse();
    Eigen::Vector3d p41_2 = T41m.block(0, 3, 3, 1);
    double p41xz_2 = hypot(p41_2(0), p41_2(2));

    T41m = T10f(th1_2).inverse() * T60 * T65f(th6_3).inverse() * T54f(th5_3).inverse();
    Eigen::Vector3d p41_3 = T41m.block(0, 3, 3, 1);
    double p41xz_3 = hypot(p41_3(0), p41_3(2));

    T41m = T10f(th1_2).inverse() * T60 * T65f(th6_4).inverse() * T54f(th5_4).inverse();
    Eigen::Vector3d p41_4 = T41m.block(0, 3, 3, 1);
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    double th3_1 = acos((p41xz_1*p41xz_1-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2)));
    double th3_2 = acos((p41xz_2*p41xz_2-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2)));
    double th3_3 = acos((p41xz_3*p41xz_3-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2)));
    double th3_4 = acos((p41xz_4*p41xz_4-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2)));
    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    double th2_1 = atan2(-p41_1(2), -p41_1(0))-asin((-A(2)*sin(th3_1))/p41xz_1);
    double th2_2 = atan2(-p41_2(2), -p41_2(0))-asin((-A(2)*sin(th3_2))/p41xz_2);
    double th2_3 = atan2(-p41_3(2), -p41_3(0))-asin((-A(2)*sin(th3_3))/p41xz_3);
    double th2_4 = atan2(-p41_4(2), -p41_4(0))-asin((-A(2)*sin(th3_4))/p41xz_4);

    double th2_5 = atan2(-p41_1(2), -p41_1(0))-asin((A(2)*sin(th3_1))/p41xz_1);
    double th2_6 = atan2(-p41_2(2), -p41_2(0))-asin((A(2)*sin(th3_2))/p41xz_2);
    double th2_7 = atan2(-p41_3(2), -p41_3(0))-asin((A(2)*sin(th3_3))/p41xz_3);
    double th2_8 = atan2(-p41_4(2), -p41_4(0))-asin((A(2)*sin(th3_4))/p41xz_4);

    Eigen::Matrix4d T43m = T32f(th3_1).inverse() * T21f(th2_1).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_1).inverse() * T54f(th5_1).inverse();
    Eigen::Vector3d Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_1 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_2).inverse() * T21f(th2_2).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_2).inverse() * T54f(th5_2).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_2 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_3).inverse() * T21f(th2_3).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_3).inverse() * T54f(th5_3).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_3 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_4).inverse() * T21f(th2_4).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_4).inverse() * T54f(th5_4).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_4 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_5).inverse() * T21f(th2_5).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_1).inverse() * T54f(th5_1).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_5 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_6).inverse() * T21f(th2_6).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_2).inverse() * T54f(th5_2).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_6 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_7).inverse() * T21f(th2_7).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_3).inverse() * T54f(th5_3).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_7 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_8).inverse() * T21f(th2_8).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_4).inverse() * T54f(th5_4).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_8 = atan2(Xhat43(1), Xhat43(0));

    Eigen::Matrix<double, 8, 6> ret;
    ret << 
        th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
        th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
        th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
        th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
        th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
        th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
        th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
        th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

    return ret;
}

Trajectory UR5::P2P(pos_rot starting, pos_rot final, uint minT, uint maxT, uint dt) {
    Trajectory ret;
    Eigen::Matrix<double, 8, 6> qEsT = IK(starting.pe, starting.Re);
    Eigen::Matrix<double, 8, 6> qEfT = IK(final.pe, final.Re);

    //qua vanno implementati limiti sul movimento

    Eigen::Vector<double, 6> qEs = qEsT.row(0);
    Eigen::Vector<double, 6> qEf = qEfT.row(0);

    Eigen::Matrix<double, 6, 4> A;
    Eigen::Matrix4d M;
    M << 
        1,  minT,   pow(minT, 2),   pow(minT, 3),
        0,  1,      2*minT,         3*pow(minT, 2),
        1,  maxT,   pow(maxT, 2),   pow(maxT, 3),
        0,  1,      2*maxT,         3*pow(maxT, 2);
    Eigen::Vector4d b, a;
    for(uint8_t i=0; i<6; ++i) {
        b << qEs[i], 0, qEf[i], 0;
        a = M.inverse()*b;
        A.row(i) << a.transpose();
    }

    ret.resize((maxT-minT)/dt);
    t_th_pos_rot ttpr;

    for(uint8_t t=minT; t<maxT; t+=dt) {
        pos_rot pr;
        ttpr.t = t;
        for(uint8_t i=0; i<6; ++i) {
            ttpr.Th[i] = A(i, 0) + A(i, 1)*t + A(i, 2)*pow(t, 2) + A(i, 3)*pow(t, 3);
        }
        pr = DK(ttpr.Th);
        ttpr.pe = pr.pe;
        ttpr.Re = pr.Re;
        ret[(t-minT)/dt] = ttpr;
    }
    return ret;
}