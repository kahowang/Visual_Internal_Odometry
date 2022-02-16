#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

using  namespace std;

int main(int argc, char const *argv[])
{
    Eigen::Vector3d  vec(1,2,3);
    cout <<  "vec:  "   <<  endl  <<  vec.transpose()  << endl  << endl;      // 自定义旋转向量
    cout << "vecnormd :"<< endl << (vec/vec.norm()).transpose() <<endl<<endl;      //  归一化
    Eigen::AngleAxisd angelaxis(M_PI/4, vec/vec.norm());      //   绕轴进行45度旋转

    //  待旋转矩阵
    Eigen::Matrix3d R = angelaxis.toRotationMatrix();               //  旋转矢量转旋转矩阵
    cout  <<  "R:  "  << endl   <<  R  << endl  << endl;

    Sophus::SO3d SO3_R(R);               // 从旋转矩阵构造Sophus::SO(3)
    Eigen::Quaterniond q(R);            // 从旋转矩阵构造四元数
    cout << "so3 :"<< endl << SO3_R.log().transpose() <<endl<<endl;
    cout << "q :"<< endl << q.coeffs().transpose() <<endl<<endl;

    Eigen::Vector3d update(0.01, 0.02, 0.03); //更新量
    // 李代数更新
    Sophus::SO3d SO3_updated = SO3_R * Sophus::SO3d::exp(update);             //右乘更新
    cout<<"SO3 updated = "<< endl << SO3_updated.matrix() <<endl<<endl;                     

    // 四元数更新
    Eigen::Quaterniond q_update(1, update(0)/2, update(1)/2, update(2)/2);
    Eigen::Quaterniond q_updated = q * q_update.normalized();           //  注意需要对更新量进行归一化
    cout<<"Quaterniond updated = "<< endl << q_updated.toRotationMatrix() <<endl<<endl;

    cout<<"error = "<< endl <<SO3_updated.matrix()-q_updated.toRotationMatrix()<<endl;

    return 0;
}
