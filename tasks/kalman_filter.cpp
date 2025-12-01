#include "kalman_filter.hpp"
#include <iostream>

// 初始化卡尔曼滤波器
KalmanFilter::KalmanFilter(double process_noise, double measure_noise) {
    // 状态量维度：6（x,y,z,vx,vy,vz），观测值维度：3（x,y,z），控制量维度：0
    kf_.init(6, 3, 0);
    // 1. 状态转移矩阵A（假设匀速运动：x(t)=x(t-1)+vx*dt，y/z同理）
    kf_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, dt_, 0, 0,
        0, 1, 0, 0, dt_, 0,
        0, 0, 1, 0, 0, dt_,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);
    // 2. 观测矩阵H（只观测位置，不观测速度）
    kf_.measurementMatrix = (cv::Mat_<float>(3, 6) <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0);
    // 3. 过程噪声协方差Q（控制状态预测的不确定性）
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(process_noise));
    // 4. 观测噪声协方差R（控制观测值的不确定性）
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(measure_noise));
    // 5. 后验误差协方差P（初始值）
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

    std::cout << "[KalmanFilter] 滤波器初始化成功，时间步长：" << dt_ << "s" << std::endl;
}

// 更新观测并预测状态
Eigen::Vector3d KalmanFilter::predictState(const Eigen::Vector3d& current_world_pos) {
    // 1. 构造观测值（当前世界坐标）
    cv::Mat measurement = (cv::Mat_<float>(3, 1) << 
        (float)current_world_pos.x(), 
        (float)current_world_pos.y(), 
        (float)current_world_pos.z());
    // 2. 预测下一帧状态
    cv::Mat predicted_state = kf_.predict();
    // 3. 用当前观测值校正预测
    cv::Mat corrected_state = kf_.correct(measurement);
    // 4. 提取预测的位置和速度
    Eigen::Vector3d predicted_pos(
        corrected_state.at<float>(0),
        corrected_state.at<float>(1),
        corrected_state.at<float>(2)
    );
    predicted_vel_ = Eigen::Vector3d(
        corrected_state.at<float>(3),
        corrected_state.at<float>(4),
        corrected_state.at<float>(5)
    );
    // 打印预测信息（验收要求：输出预测后的状态值）
    std::cout << "[KalmanFilter] 预测状态 - 位置：(" 
              << predicted_pos.x() << "," << predicted_pos.y() << "," << predicted_pos.z() << ")m，"
              << "速度：(" << predicted_vel_.x() << "," << predicted_vel_.y() << "," << predicted_vel_.z() << ")m/s" << std::endl;
    return predicted_pos;
}