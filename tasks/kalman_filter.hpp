#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// 卡尔曼滤波器（进阶要求3：预测装甲板运动状态）
class KalmanFilter {
public:
    // 初始化滤波器（状态量：[x, y, z, vx, vy, vz]，观测值：[x, y, z]）
    KalmanFilter(double process_noise = 1e-4, double measure_noise = 1e-2);
    // 更新观测值并预测下一帧状态
    Eigen::Vector3d predictState(const Eigen::Vector3d& current_world_pos);
    // 获取当前预测的速度
    Eigen::Vector3d getPredictedVel() const { return predicted_vel_; }

private:
    cv::KalmanFilter kf_;         // OpenCV卡尔曼滤波对象
    Eigen::Vector3d predicted_vel_;  // 预测的速度（vx, vy, vz）
    double dt_ = 0.1;             // 时间步长（默认100ms，与主循环帧率匹配）
};

#endif // KALMAN_FILTER_HPP