#include "modern_robotics.h"
#include <iostream>
#include <Eigen/Geometry>

using namespace mr;
using namespace std;
int main()
{
    Eigen::Vector3d w1(0, 0, 1), w2(0, 0, 1), w3(0, 0, 1); //角速度方向
    Eigen::Vector3d q1(0, 0, 0), q2(1, 0, 0), q3(2, 0, 0); //螺旋轴位置
    Eigen::VectorXd s1(6, 1), s2(6, 1), s3(6, 1);          //螺旋轴序列
    s1 << w1, -w1.cross(q1);
    s2 << w2, -w2.cross(q2);
    s3 << w3, -w3.cross(q3);
    Eigen::MatrixXd Slist(6, 3);
    Slist << s1, s2, s3;
    cout << "Slist=" << endl;
    cout << Slist << endl;

    Eigen::MatrixXd M(4, 4); //零位时末端的位姿矩阵
    M << 1, 0, 0, 2,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    cout << "M = " << endl;
    cout << M << endl;

    Eigen::MatrixXd T(4, 4); //目标位姿矩阵
    T << 1, 0, 0, 1,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
    cout << "T = " << endl;
    cout << T << endl;
    cout << endl;

    Eigen::VectorXd thetalist(3, 1); //关节角度值
    thetalist << 0, M_PI / 3, 0;
    cout << "initial joint = " << endl;
    cout << thetalist << endl;
    double eomg = 0.0;
    double ev = 0.0;
    IKinSpace(Slist, M, T, thetalist, eomg, ev);
    cout << "joint result = " << endl;
    cout << thetalist << endl;
    cout << "FKinSpace = " << endl;
    cout << FKinSpace(M, Slist, thetalist) << endl;
    cout << "FKinSpace = T" << endl;
}