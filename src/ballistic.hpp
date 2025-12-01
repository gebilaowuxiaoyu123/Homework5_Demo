#ifndef BALLISTIC_HPP
#define BALLISTIC_HPP

#include <Eigen/Core>
#include "../io/camera_io.hpp"  // 依赖相机参数

// 弹道解算核心类（计算yaw/pitch角度，考虑空气阻力）
class BallisticCalculator {
public:
    // 初始化解算器（输入弹丸参数）
    BallisticCalculator(double bullet_mass = 0.01,    // 弹丸质量(kg)
                       double bullet_drag_coeff = 0.295,  // 曳力系数（球形弹丸默认值）
                       double bullet_radius = 0.006); // 弹丸半径(m)
    // 计算命中目标所需的yaw和pitch角度
    void calculateYawPitch(const Eigen::Vector3d& target_world_pos, 
                          double bullet_speed,  // 弹丸速度(m/s，给定速度区间内)
                          const CameraParams& params,
                          double& yaw,         // 输出yaw角(°)
                          double& pitch);      // 输出pitch角(°)

private:
    // 空气阻力计算（基于曳力公式：F=0.5*ρ*v²*S*Cd，ρ为空气密度）
    Eigen::Vector3d calculateDragForce(const Eigen::Vector3d& bullet_vel);

    // 物理参数（固定值）
    const double RHO_ = 1.225;    // 空气密度(kg/m³，标准大气压)
    const double G_ = 9.81;       // 重力加速度(m/s²)
    double m_;                    // 弹丸质量
    double Cd_;                   // 曳力系数
    double S_;                    // 弹丸迎风面积(m²，S=πr²)
};

#endif // BALLISTIC_HPP