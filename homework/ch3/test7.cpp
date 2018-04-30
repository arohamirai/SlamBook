#include <iostream>
#include <iomanip>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <pangolin/pangolin.h>

int main(int argc, char** argv)
{
    Eigen::Vector3f posInRobot1(0.5,0,0.2,1);
    Eigen::Quaternionf
            robot1_q(0.35,0.2, 0.3, 0.1),
            robot2_q(-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3f
            robot1_t(0.3, 0.1, 0.1),
            robot2_t(-0.1, 0.5, 0.3);

    Eigen::Matrix3f R1,R2;
    R1 = robot1_q.matrix();
    R2 = robot2_q.matrix();

    Eigen::Matrix4f T1,T2;
    T1.block<3,3>(0,0) = R1;
    T1.block<3,1>(0,3) = robot1_t;

    T2.block<3,3>(0,0) = R2;
    T2.block<3,1>(0,3) = robot2_t;

    Eigen::Vector3f posInRobot2 = T2 * Eigen::inverse(T1)*posInRobot1;
    cout<<"posInRobot2:\n"<<posInRobot2<<endl;
}
