#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度
    cout .precision(3);         //打印精度

    // 1.0 旋转向量————> 旋转矩阵
    //用matrix()转换成矩阵
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;
    // 也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();
    // 1.1 旋转矩阵————> 旋转向量
    rotation_vector.fromRotationMatrix(rotation_matrix);

    // 2.0 旋转矩阵————>欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;
    // 2.1 欧拉角————>旋转矩阵
    // 欧拉角——>旋转向量——>旋转矩阵


    // 3.0 旋转向量————>四元数
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    // 3.1 四元数————> 旋转向量
    // 四元数——>旋转矩阵——>旋转向量


    // 4.0 旋转矩阵————>四元数
    q = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;
    // 4.1 四元数————>旋转矩阵
    rotation_matrix = q.toRotationMatrix();


    // 变换
    // 1. 用 AngleAxis 可以进行坐标变换
    Eigen::Vector3d v ( 1,0,0 );
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    // 2. 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    // 3. 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q*v; // 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;




}
