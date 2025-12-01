#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include "../io/camera_io.hpp"  // 依赖相机参数
#include "../tools/img_tools.hpp"  // 依赖图像工具


// 装甲板6D位姿结构体（验收要求：输出6D位姿）
struct ArmorPose {
    cv::RotatedRect armor_rect;  // 装甲板旋转矩形（框选结果）
    Eigen::Vector3d cam_pos;     // 相机坐标系下中心位置
    Eigen::Vector3d world_pos;   // 世界坐标系下中心位置
    Eigen::Vector3d euler;       // 姿态角（roll, pitch, yaw，单位：°）
    double depth;                // 装甲板到相机的深度（单位：m）
};

// 装甲板检测任务（基于tiny_resnet.onnx模型）
class ArmorDetector {
public:
    // 初始化检测器（加载ONNX模型）
    ArmorDetector(const std::string& onnx_path = "tiny_resnet.onnx");
    ~ArmorDetector();
    // 检测装甲板并输出6D位姿
    ArmorPose detectArmor(const cv::Mat& undistorted_img, const CameraParams& params);

private:
    Ort::Env env_;                // ONNX Runtime环境
    Ort::SessionOptions session_opt_;  // 会话配置
    Ort::Session* session_;       // 推理会话
    std::vector<std::string> input_names_;  // 输入节点名
    std::vector<std::string> output_names_; // 输出节点名
    int model_input_c_ = 3;
    int model_input_w_ = 640;     // 模型输入宽度
    int model_input_h_ = 480;     // 模型输入高度

    // 图像预处理（归一化、HWC→CHW）
    Ort::Value preprocessImage(const cv::Mat& img);
    // 解析模型输出（得到旋转矩形、深度、姿态角）
    ArmorPose parseModelOutput(const Ort::Value& output, 
                               const cv::Mat& src_img, const CameraParams& params);
};

#endif // ARMOR_DETECTOR_HPP