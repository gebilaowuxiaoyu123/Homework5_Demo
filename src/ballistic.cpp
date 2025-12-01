#include "ballistic.hpp"
#include <cmath>
#include <iostream>

// 初始化弹道解算器（设置弹丸参数）
BallisticCalculator::BallisticCalculator(double bullet_mass, double bullet_drag_coeff, double bullet_radius) {
    m_ = bullet_mass;
    Cd_ = bullet_drag_coeff;
    S_ = M_PI * bullet_radius * bullet_radius;  // 迎风面积=πr²
    std::cout << "[BallisticCalculator] 解算器初始化成功 - "
              << "弹丸质量：" << m_ << "kg，"
              << "曳力系数：" << Cd_ << "，"
              << "迎风面积：" << S_ << "m²" << std::endl;
}

// 计算空气阻力
Eigen::Vector3d BallisticCalculator::calculateDragForce(const Eigen::Vector3d& bullet_vel) {
    double vel_mag = bullet_vel.norm();  // 速度大小
    if (vel_mag < 1e-6) {  // 速度为0时无阻力
        return Eigen::Vector3d::Zero();
    }
    // 曳力公式：F_d = -0.5*ρ*v²*S*Cd * (v/|v|)（与速度方向相反）
    double drag_mag = 0.5 * RHO_ * vel_mag * vel_mag * S_ * Cd_;
    Eigen::Vector3d drag_dir = -bullet_vel.normalized();
    return drag_dir * drag_mag;
}

// 计算yaw和pitch角度
void BallisticCalculator::calculateYawPitch(const Eigen::Vector3d& target_world_pos, 
                                           double bullet_speed, 
                                           const CameraParams& params,
                                           double& yaw, 
                                           double& pitch) {
    // 1. 计算目标相对于机器人的位置（世界系下，机器人原点为参考）
    Eigen::Vector3d target_rel = target_world_pos - params.t_cam_robot;  // 机器人系原点默认与相机重合
    double target_dist = target_rel.norm();  // 目标距离
    if (target_dist < 0.1) {  // 距离过近，无需计算
        yaw = 0.0;
        pitch = 0.0;
        return;
    }

    // 2. 计算yaw角（绕Z轴旋转，水平方向）
    // yaw = arctan2(目标y坐标, 目标x坐标)（右手系，逆时针为正）
    yaw = atan2(target_rel.y(), target_rel.x()) * 180.0 / M_PI;

    // 3. 计算pitch角（绕Y轴旋转，垂直方向，考虑重力和阻力）
    // 简化模型：假设水平方向匀速，垂直方向受重力和阻力
    double horizontal_dist = sqrt(target_rel.x()*target_rel.x() + target_rel.y()*target_rel.y());  // 水平距离
    double time_of_flight = horizontal_dist / (bullet_speed * cos(yaw * M_PI / 180.0));  // 飞行时间（水平匀速近似）
    // 垂直方向位移：目标z坐标 = 初始垂直速度*时间 - 0.5*g*t²（忽略阻力简化）
    double vertical_vel = (target_rel.z() + 0.5 * G_ * time_of_flight * time_of_flight) / time_of_flight;
    // pitch = arctan2(垂直速度, 水平速度)
    pitch = atan2(vertical_vel, bullet_speed * cos(yaw * M_PI / 180.0)) * 180.0 / M_PI;

    // 打印弹道信息（验收要求：输出需要转动的yaw与pitch角度）
    std::cout << "[BallisticCalculator] 弹道解算结果 - "
              << "目标距离：" << target_dist << "m，"
              << "飞行时间：" << time_of_flight << "s，"
              << "Yaw：" << yaw << "°，Pitch：" << pitch << "°" << std::endl;
}