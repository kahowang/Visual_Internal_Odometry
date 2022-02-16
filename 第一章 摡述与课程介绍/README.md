# 从零开始手写VIO  第一章 概述与课程介绍

课程代码：

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

参考博客：

1.作业参考 ：深蓝学院《从零开始手写VIO》作业1](https://blog.csdn.net/hitljy/article/details/107320682)

2.李群、李代数、左扰动、右扰动复习回顾：[视觉SLAM十四讲：李群李代数](https://www.guyuehome.com/18566)

## 1.VIO 文献阅读

因为博客 [深蓝学院《从零开始手写VIO》作业1](https://blog.csdn.net/hitljy/article/details/107320682)已总结很充分了，所以这部分摘抄自该大佬博客

### 1.1 视觉与IMU进行融合之后有何优势

| 方案 | IMU                                                          | 视觉                                                         |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 优势 | 快速响应；<br/>不受成像质量影响；<br/>角速度普遍比较准确；<br/>可估计绝对尺度 | 不产生漂移；<br/>直接测量旋转与平移                          |
| 劣势 | 存在零偏；<br/>低精度 IMU 积分位姿发散；<br/>高精度价格昂贵  | 受图像遮挡、运动物体干扰; <br/>单目视觉无法测量尺度; <br/>单目纯旋转运动无法估计; <br/>快速运动时易丢失 |

以下摘自《视觉 SLAM 十四讲》：

​		1.MU虽然可以测得角速度和加速度，但这些量都存在明显的漂移（Drift），使得积分两次得到的位姿数据非常不可靠。好比说，我们将IMU放在桌上不动，用它的读数积分得到的位姿也会漂出十万八千里。但是，对于短时间内的快速运动，IMU能够提供一些较好的估计。这正是相机的弱点。当运动过快时，（卷帘快门的）相机会出现运动模糊，或者两帧之间重叠区域太少以至于无法进行特征匹配，所以纯视觉SLAM非常害怕快速的运动。而有了IMU，即370第14讲SLAM：现在与未来使在相机数据无效的那段时间内，我们还能保持一个较好的位姿估计，这是纯视觉SLAM无法做到的。
​		2.相比于IMU，相机数据基本不会有漂移。如果相机放在原地固定不动，那么（在静态场景下）视觉SLAM的位姿估计也是固定不动的。所以，相机数据可以有效地估计并修正IMU读数中的漂移，使得在慢速运动后的位姿估计依然有效。
​		3.当图像发生变化时，本质上我们没法知道是相机自身发生了运动，还是外界条件发生了变化，所以纯视觉SLAM难以处理动态的障碍物。而IMU能够感受到自己的运动信息，从某种程度上减轻动态物体的影响。
​		总而言之，我们看到IMU为快速运动提供了较好的解决方式，而相机又能在慢速运动下解决IMU的漂移问题——在这个意义下，它们二者是互补的。
​		整体上，视觉和 IMU 定位方案存在一定互补性质：
​	• IMU 适合计算短时间、快速的运动；
​	• 视觉适合计算长时间、慢速的运动。
同时，可利用视觉定位信息来估计 IMU 的零偏，减少 IMU 由零偏导致的发散和累积误差；反之， IMU 可以为视觉提供快速运动时的定位

视觉与IMU融合之后会弥补各自的劣势，可利用视觉定位信息来减少IMU由零偏导致的发散和累积误差；IMU可以为视觉提供快速运动时的定位，减少因为外界影响定位失败，同时有效解决单目尺度不可观测的问题

### 1.2  有哪些常见的视觉 +IMU 融合方案？有没有工业界应用的例子？

VINS (单目+IMU、双目+IMU)
OKVIS (单目+IMU、双目+IMU)
ROVIO (单目+IMU）
RKSLAM (单目+IMU）
ORB_SLAM-IMU（单目+IMU）

AR/VR，自动驾驶，无人机，手机、无人机拍照防抖

### 1.3  在学术界， VIO 研究有哪些新进展？有没有将学习方法用到 VIO中的例子？

[LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping](TixiaoShan/LVI-SAM: LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry
via Smoothing and Mapping (github.com))

[R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual tightly-coupled state Estimator and mapping]()

## 2.四元数和李代数更新

![problem2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11problem2.png)

四元数更新 
$$
\begin{equation}
\begin{array}{ll} 
& \mathbf{q} \leftarrow \mathbf{q} \otimes\left[1, \frac{1}{2} \boldsymbol{\omega}\right]^{\top}
\end{array}
\end{equation}
$$
旋转矩阵更新
$$
\begin{equation}
\begin{array}{ll} 
& \mathbf{R} \leftarrow \mathbf{R} \exp \left(\boldsymbol{\omega}^{\wedge}\right) \\
\end{array}
\end{equation}
$$
FILE:   update_Quaterniond_SO3.cpp

本次调用sophus中的SO3进行旋转矩阵更新

```cpp
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
```

![problem2_2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11problem2_2.png)

FILE : CMakeLists

```shell
#CMakeLists.txt
cmake_minimum_required(VERSION 3.0)
project(update_Quaterniond_SO3)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")
add_executable(update_Quaterniond_SO3 src/update_Quaterniond_SO3.cpp)
target_link_libraries(update_Quaterniond_SO3 Sophus::Sophus)
```

注意：因为我们构造的四元数的四元数[1, 0.05, 0.01, 0.015 ] 幷不是个单位四元数，需要归一化。

## 3. 使用右乘so(3),推导以下导数：

![problem3](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11problem3.png)

### 3.1  第一题

均参考 [深蓝学院《从零开始手写VIO》作业1](https://blog.csdn.net/hitljy/article/details/107320682)

李群性质

李群性质
$$
\begin{array}{c}
\boldsymbol{C}=\exp \left(\phi^{\wedge}\right) \equiv \sum_{n=0}^{\infty} \frac{1}{n !}\left(\phi^{\wedge}\right)^{n} \\
\equiv \cos \phi \mathbf{1}+(1-\cos \phi) \boldsymbol{a} \boldsymbol{a}^{T}+\sin \phi \boldsymbol{a}^{\wedge} \\
\approx \mathbf{1}+\phi^{\wedge} \\
\boldsymbol{C}^{-1} \equiv \boldsymbol{C}^{\mathrm{T}} \equiv \sum_{n=0}^{\infty} \frac{1}{n !}\left(-\boldsymbol{\phi}^{\wedge}\right)^{n} \approx \mathbf{1}-\boldsymbol{\phi}^{\wedge}
\end{array}
$$

$$
\begin{aligned}
\frac{d\left(R^{-1} p\right)}{d R} &=\lim _{\phi \rightarrow 0} \frac{\left(R \cdot \exp \left(\phi^{\wedge}\right)\right)^{-1} \cdot p-R^{-1} p}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\left(\exp \left(\phi^{\wedge}\right)\right)^{-1} R^{-1} \cdot p-R^{-1} p}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\exp \left(-\phi^{\wedge}\right) \cdot R^{-1} \cdot p-R^{-1} p}{\phi} \\
& \approx \lim _{\phi \rightarrow 0} \frac{\left(I-\phi^{\wedge}\right) \cdot R^{-1} \cdot p-R^{-1} \cdot p}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{-\phi^{\wedge} R^{-1} p}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\phi \cdot\left(R^{-1} p\right)^{\wedge}}{\phi} \\
&=\left(R^{-1} p\right)^{\wedge}
\end{aligned}
$$

### 3.2  第二题

$$
\begin{array}{c}
(\boldsymbol{C u})^{\wedge} \equiv \boldsymbol{C u}^{\wedge} \boldsymbol{C}^{\mathrm{T}} \\
\exp \left((\boldsymbol{C u})^{\wedge}\right) \equiv \boldsymbol{C} \exp \left(\boldsymbol{u}^{\wedge}\right) \boldsymbol{C}^{\mathrm{T}} \\
\ln \left(\exp \left(\boldsymbol{\phi}_{1}^{\wedge}\right) \exp \left(\boldsymbol{\phi}_{2}^{\wedge}\right)\right)^{\vee} \approx\left\{\begin{array}{l}
\boldsymbol{J}_{l}\left(\boldsymbol{\phi}_{2}\right)^{-1} \boldsymbol{\phi}_{1}+\boldsymbol{\phi}_{2}   &当 \boldsymbol{\phi}_{1}为小量\\
\boldsymbol{J}_{r}\left(\boldsymbol{\phi}_{1}\right)^{-1} \boldsymbol{\phi}_{2}+\boldsymbol{\phi}_{1}&当 \boldsymbol{\phi}_{2}为小量
\end{array}\right.
\end{array}
$$

$$
\begin{aligned}
\frac{d \ln \left(R_{1} R_{2}^{-1}\right)^{\vee}}{d R_{2}} &=\lim _{\phi \rightarrow 0} \frac{\ln \left(R_{1}\left(R_{2} \cdot \exp \left(\phi^{\wedge}\right)\right)^{-1}\right)^{\vee}-\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\ln \left(R_{1} \cdot \exp \left(-\phi^{\wedge}\right) \cdot R_{2}^{-1}\right)^{\vee}-\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\ln \left(R_{1} \cdot R_{2}^{-1} \cdot R_{2} \cdot \exp \left(-\phi^{\wedge}\right) \cdot R_{2}^{-1}\right)^{\vee}-\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\ln \left(R_{1} \cdot R_{2}^{-1} \cdot \exp \left(\left(-R_{2} \phi\right)^{\wedge}\right)\right)^{\vee}-\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{\ln \left(\exp \left(\ln \left(\left(R_{1} R_{2}^{-1}\right)^{\vee}\right) \cdot \exp \left(\left(-R_{2} \phi\right)^{\wedge}\right)\right)^{\vee}-\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}\right.}{\phi} \\
&=\lim _{\phi \rightarrow 0} \frac{J_{r}\left(\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}\right)^{-1} \cdot\left(-R_{2} \phi\right)+\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}-\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}}{\phi} \\
&=-J_{r}\left(\ln \left(R_{1} R_{2}^{-1}\right)^{\vee}\right)^{-1} \cdot R_{2}
\end{aligned}
$$

​																																									edited by  kaho  2022.2.16