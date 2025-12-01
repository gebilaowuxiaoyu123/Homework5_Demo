#ifndef CAMERA_IO_HPP
#define CAMERA_IO_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// 相机内参与外参结构体（匹配内参.txt参数）
struct CameraParams {
    Eigen::Matrix3d K;          // 内参矩阵
    Eigen::Vector3d dist;       // 畸变系数
    Eigen::Matrix3d R_cam_robot;// 相机→机器人系旋转矩阵（REP-105标准）
    Eigen::Vector3d t_cam_robot;// 相机→机器人系平移向量
    double fx, fy, cx, cy;      // 内参拆解（方便计算）
    int img_width = 3072;       // 图像宽度（内参.txt中width=3072）
    int img_height = 2048;      // 图像高度（内参.txt中height=2048）
};

class CameraIO {
public:
    // 加载相机参数（从内参.txt读取预设值）
    CameraParams loadCameraParams();
    // 初始化工业相机（MV-CS060-10UC-PRO）
    bool initCamera(int cam_idx = 0);
    // 采集一帧图像并做畸变校正
    cv::Mat captureUndistortedFrame(const CameraParams& params);
    // 释放相机资源
    void releaseCamera() { cap_.release(); }

private:
    cv::VideoCapture cap_;  // 相机捕获对象
};

#endif // CAMERA_IO_HPP