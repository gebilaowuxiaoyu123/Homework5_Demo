#include <iostream>
#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include "../io/camera_io.hpp"
#include "../io/serial_io.hpp"
#include "../tasks/armor_detector.hpp"
#include "../tasks/kalman_filter.hpp"
#include "../tools/img_tools.hpp"
#include "../tools/coord_tools.hpp"
#include "ballistic.hpp"

int main(int argc, char** argv) {
    // -------------------------- 1. 初始化各模块 --------------------------
    CameraIO camera_io;                // 相机IO
    SerialIO serial_io;                // 串口IO
    ArmorDetector armor_detector;      // 装甲板检测器
    KalmanFilter kalman_filter;        // 卡尔曼滤波器
    BallisticCalculator ballistic_calc;// 弹道解算器

    // 加载相机参数（内参.txt预设值）
    CameraParams cam_params = camera_io.loadCameraParams();
    // 初始化工业相机
    if (!camera_io.initCamera(0)) {
        std::cerr << "[Main] 相机初始化失败，程序退出！" << std::endl;
        return -1;
    }

    // -------------------------- 2. 主循环（图像采集→处理→输出） --------------------------
    const double bullet_speed = 30.0;  // 弹丸速度（可配置在给定速度区间内）
    cv::Mat frame;
    std::cout << "[Main] 主循环启动，弹丸速度：" << bullet_speed << "m/s，按ESC退出" << std::endl;
    while (true) {
        // 2.1 采集畸变校正后的图像
        frame = camera_io.captureUndistortedFrame(cam_params);
        if (frame.empty()) {
            std::cerr << "[Main] 图像采集失败，跳过当前帧！" << std::endl;
            continue;
        }

        // 2.2 装甲板检测（输出6D位姿）
        ArmorPose armor_pose = armor_detector.detectArmor(frame, cam_params);
        if (armor_pose.armor_rect.size.width <= 0) {
            drawImageText(frame, "Armor Not Detected", cv::Point(20, 40), cv::Scalar(0, 0, 255));
            cv::imshow("Ballistic Calculation", frame);
            if (cv::waitKey(1) == 27) break;
            continue;
        }

        // 2.3 验证进阶要求1：pitch旋转稳定性（仅首次检测时执行）
        static bool verify_pitch = true;
        if (verify_pitch) {
            verifyPitchStability(armor_pose.cam_pos, cam_params);
            verify_pitch = false;  // 仅验证一次
        }

        // 2.4 卡尔曼滤波预测装甲板状态
        Eigen::Vector3d predicted_target_pos = kalman_filter.predictState(armor_pose.world_pos);

        // 2.5 弹道解算：计算yaw/pitch角度
        double yaw, pitch;
        ballistic_calc.calculateYawPitch(predicted_target_pos, bullet_speed, cam_params, yaw, pitch);

        // 2.6 串口发送yaw/pitch指令（进阶要求2）
        if (serial_io.isOpen()) {
            if (serial_io.sendYawPitch(yaw, pitch)) {
                drawImageText(frame, "Serial Send: Success", cv::Point(20, 120), cv::Scalar(0, 255, 0));
            } else {
                drawImageText(frame, "Serial Send: Failed", cv::Point(20, 120), cv::Scalar(0, 0, 255));
            }
        } else {
            drawImageText(frame, "Serial: Not Open", cv::Point(20, 120), cv::Scalar(0, 0, 255));
        }

        // 2.7 上位机显示（验收要求：框选装甲板、输出6D位姿和角度）
        drawArmorRect(frame, armor_pose.armor_rect);  // 框选装甲板
        // 绘制6D位姿信息
        std::string pose_text = fmt::format(
            "6D Pose: x={:.2f}, y={:.2f}, z={:.2f}m, roll={:.1f}, pitch={:.1f}, yaw={:.1f}°",
            armor_pose.world_pos.x(), armor_pose.world_pos.y(), armor_pose.world_pos.z(),
            armor_pose.euler.x(), armor_pose.euler.y(), armor_pose.euler.z()
        );
        drawImageText(frame, pose_text, cv::Point(20, 40));
        // 绘制yaw/pitch角度
        std::string angle_text = fmt::format("Yaw={:.2f}°, Pitch={:.2f}°", yaw, pitch);
        drawImageText(frame, angle_text, cv::Point(20, 80), cv::Scalar(0, 0, 255));

        // 显示窗口
        cv::imshow("Ballistic Calculation", frame);
        // 按ESC退出循环
        if (cv::waitKey(1) == 27) break;
    }

    // -------------------------- 3. 释放资源 --------------------------
    camera_io.releaseCamera();
    cv::destroyAllWindows();
    std::cout << "[Main] 程序正常退出！" << std::endl;
    return 0;
}