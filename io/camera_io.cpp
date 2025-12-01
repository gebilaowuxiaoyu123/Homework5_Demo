#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "camera_io.hpp"
//#include <opencv2/core/eigen.hpp>
#include <iostream>

// 加载内参.txt中的预设参数
CameraParams CameraIO::loadCameraParams() {
    CameraParams params;
    // 内参矩阵（匹配内参.txt中[narrow_stereo]的camera matrix）
    params.K << 3459.981717, 0.000000, 1523.076759,
                0.000000, 3469.145301, 1019.198285,
                0.000000, 0.000000, 1.000000;
    // 畸变系数（匹配内参.txt中的distortion）
    params.dist << -0.091955, 0.395148, 0.003058, -0.000456, 0.000000;
    // 相机→机器人系外参（默认重合，可根据实际校准修改）
    params.R_cam_robot.setIdentity();
    params.t_cam_robot.setZero();
    // 拆解内参
    params.fx = params.K(0, 0);
    params.fy = params.K(1, 1);
    params.cx = params.K(0, 2);
    params.cy = params.K(1, 2);
    return params;
}

// 初始化相机（指定工业相机型号MV-CS060-10UC-PRO）
bool CameraIO::initCamera(int cam_idx) {
    cap_.open(cam_idx, cv::CAP_V4L2);  // 工业相机优先用V4L2驱动
    if (!cap_.isOpened()) {
        std::cerr << "[CameraIO] 工业相机（MV-CS060-10UC-PRO）打开失败！" << std::endl;
        return false;
    }
    // 设置图像分辨率（匹配内参.txt的width/height）
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 3072);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 2048);
    std::cout << "[CameraIO] 相机初始化成功，分辨率：" << 3072 << "x" << 2048 << std::endl;
    return true;
}

// 采集畸变校正后的图像
cv::Mat CameraIO::captureUndistortedFrame(const CameraParams& params) {
    cv::Mat frame, undistorted_frame;
    cap_.read(frame);
    if (frame.empty()) {
        std::cerr << "[CameraIO] 图像采集失败！" << std::endl;
        return undistorted_frame;
    }
    // 畸变校正（使用内参和畸变系数）
    cv::Mat cv_K = cv::Mat(3, 3, CV_64F, const_cast<double*>(params.K.data()));
    cv::Mat cv_dist = cv::Mat(1, 5, CV_64F, const_cast<double*>(params.dist.data()));
    cv::undistort(frame, undistorted_frame, cv_K, cv_dist);
    return undistorted_frame;
}