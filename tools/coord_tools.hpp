#ifndef COORD_TOOLS_HPP
#define COORD_TOOLS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "../io/camera_io.hpp"  // 依赖相机参数结构体

// 图像坐标→相机坐标系（反投影：已知深度）
inline Eigen::Vector3d img2CamCoord(const cv::Point2f& img_pt, double depth, const CameraParams& params) {
    // 公式：X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy, Z=depth
    double X = (img_pt.x - params.cx) * depth / params.fx;
    double Y = (img_pt.y - params.cy) * depth / params.fy;
    double Z = depth;
    return Eigen::Vector3d(X, Y, Z);
}

// 相机坐标系→世界坐标系（进阶要求1：定轴坐标系转换，抵消云台pitch旋转影响）
inline Eigen::Vector3d cam2WorldCoord(const Eigen::Vector3d& cam_pt, double pitch_offset, const CameraParams& params) {
    // 1. 云台pitch旋转矩阵（绕Y轴旋转）
    Eigen::AngleAxisd pitch_rot(pitch_offset * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d R_pitch = pitch_rot.toRotationMatrix();
    // 2. 相机系→机器人系（考虑pitch旋转）
    Eigen::Vector3d robot_pt = params.R_cam_robot * R_pitch * cam_pt + params.t_cam_robot;
    // 3. 机器人系→世界系（定轴坐标系，默认机器人系与世界系重合）
    Eigen::Matrix3d R_robot_world = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_robot_world = Eigen::Vector3d::Zero();
    return R_robot_world * robot_pt + t_robot_world;
}

// 验证进阶要求1：pitch旋转-15°/0°/15°时世界坐标稳定性
inline void verifyPitchStability(const Eigen::Vector3d& cam_pt, const CameraParams& params) {
    Eigen::Vector3d world_0 = cam2WorldCoord(cam_pt, 0.0, params);    // pitch=0°
    Eigen::Vector3d world_15 = cam2WorldCoord(cam_pt, 15.0, params);  // pitch=15°
    Eigen::Vector3d world_neg15 = cam2WorldCoord(cam_pt, -15.0, params); // pitch=-15°
    // 计算坐标误差（验收要求：xyz基本不变，误差<0.1m）
    double err_15 = (world_15 - world_0).norm();
    double err_neg15 = (world_neg15 - world_0).norm();
    std::cout << "[CoordTools] Pitch旋转稳定性验证：" << std::endl;
    std::cout << "  pitch=0°: (" << world_0.x() << "," << world_0.y() << "," << world_0.z() << ")" << std::endl;
    std::cout << "  pitch=15°: 误差=" << err_15 << "m" << std::endl;
    std::cout << "  pitch=-15°: 误差=" << err_neg15 << "m" << std::endl;
}

#endif // COORD_TOOLS_HPP