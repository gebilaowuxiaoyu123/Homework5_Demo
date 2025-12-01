#include "armor_detector.hpp"
#include "../tools/coord_tools.hpp"  // 依赖坐标转换工具
#include <iostream>
#include <onnxruntime_cxx_api.h>

// 修正点1：用成员初始化列表初始化Ort::Env
ArmorDetector::ArmorDetector(const std::string& onnx_path) 
    : env_(ORT_LOGGING_LEVEL_WARNING, "ArmorDetector")  // 初始化列表初始化env_
{
    session_opt_.SetIntraOpNumThreads(4);  // 线程数
    // 修正点2：添加Ort::命名空间
    session_opt_.SetGraphOptimizationLevel(ORT_ENABLE_BASIC); // 基础优化

    // 加载模型
    try {
        session_ = new Ort::Session(env_, onnx_path.c_str(), session_opt_);
    } catch (const Ort::Exception& e) {
        std::cerr << "[ArmorDetector] ONNX模型加载失败：" << e.what() << std::endl;
        exit(-1);
    }

    // 获取输入输出节点名
    input_names_ = {"input"};  // 模型输入节点名（需与训练时一致）
    output_names_ = {"armor_info"};  // 模型输出节点名（包含矩形、深度、姿态角）

    // 获取模型输入尺寸
    auto input_info = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
    auto input_shape = input_info.GetShape();
    if (input_shape.size() != 4) {
        std::cerr << "[ArmorDetector] 模型输入维度错误（需NCHW格式）！" << std::endl;
        exit(-1);
    }
    model_input_c_ = input_shape[1];  // 通道数（默认3）
    model_input_h_ = input_shape[2];  // 高度
    model_input_w_ = input_shape[3];  // 宽度
    std::cout << "[ArmorDetector] ONNX模型加载成功，输入尺寸：" 
              << model_input_w_ << "x" << model_input_h_ << "x" << model_input_c_ << std::endl;
}

// 释放模型资源
ArmorDetector::~ArmorDetector() {
    if (session_ != nullptr) {
        delete session_;
    }
}

// 图像预处理（适配ONNX模型输入）
Ort::Value ArmorDetector::preprocessImage(const cv::Mat& img) {
    // 1. 准备内存信息
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    
    // 2. 构造输入形状（NCHW：1批次、C通道、H高度、W宽度）
    std::vector<int64_t> input_shape = {1, model_input_c_, model_input_h_, model_input_w_};
    
    // 3. 填充输入数据（修正点4：补充图像预处理逻辑）
    cv::Mat resized, normalized;
    cv::resize(img, resized, cv::Size(model_input_w_, model_input_h_));  // 缩放到模型输入尺寸
    resized.convertTo(normalized, CV_32F, 1.0 / 255.0);  // 归一化到[0,1]
    // HWC转CHW（OpenCV是HWC，模型是CHW）
    std::vector<float> input_data(model_input_c_ * model_input_h_ * model_input_w_);
    int idx = 0;
    for (int c = 0; c < model_input_c_; ++c) {
        for (int h = 0; h < model_input_h_; ++h) {
            for (int w = 0; w < model_input_w_; ++w) {
                input_data[idx++] = normalized.at<cv::Vec3f>(h, w)[c];
            }
        }
    }
    
    // 4. 创建Ort::Value张量
    return Ort::Value::CreateTensor<float>(
        memory_info,
        input_data.data(),
        input_data.size(),
        input_shape.data(),
        input_shape.size()
    );
}

// 解析模型输出，生成装甲板6D位姿
ArmorPose ArmorDetector::parseModelOutput(const Ort::Value& output, 
                                         const cv::Mat& src_img, const CameraParams& params) {
    ArmorPose armor_pose;
    auto output_data = output.GetTensorData<float>();
    if (output_data == nullptr) {
        std::cerr << "[ArmorDetector] 模型输出解析失败！" << std::endl;
        return armor_pose;
    }

    // 模型输出格式（共9个参数）：[x, y, w, h, angle, depth, roll, pitch, yaw]
    int output_idx = 0;
    float x = output_data[output_idx++] * src_img.cols / model_input_w_;  // 缩放回原图尺寸
    float y = output_data[output_idx++] * src_img.rows / model_input_h_;
    float w = output_data[output_idx++] * src_img.cols / model_input_w_;
    float h = output_data[output_idx++] * src_img.rows / model_input_h_;
    float angle = output_data[output_idx++];
    armor_pose.depth = output_data[output_idx++];
    armor_pose.euler.x() = output_data[output_idx++];
    armor_pose.euler.y() = output_data[output_idx++];
    armor_pose.euler.z() = output_data[output_idx++];

    // 构建旋转矩形+坐标转换
    armor_pose.armor_rect = cv::RotatedRect(cv::Point2f(x, y), cv::Size2f(w, h), angle);
    armor_pose.cam_pos = img2CamCoord(armor_pose.armor_rect.center, armor_pose.depth, params);
    armor_pose.world_pos = cam2WorldCoord(armor_pose.cam_pos, 0.0, params);

    return armor_pose;
}

// 检测装甲板主函数
ArmorPose ArmorDetector::detectArmor(const cv::Mat& undistorted_img, const CameraParams& params) {
    if (undistorted_img.empty()) {
        std::cerr << "[ArmorDetector] 输入图像为空！" << std::endl;
        return ArmorPose();
    }
    // 1. 图像预处理
    Ort::Value input_tensor = preprocessImage(undistorted_img);
    // 修正点3：将节点名转为const char*数组（匹配Run的参数要求）
    std::vector<const char*> input_names_cstr;
    for (const auto& name : input_names_) {
        input_names_cstr.push_back(name.c_str());
    }
    std::vector<const char*> output_names_cstr;
    for (const auto& name : output_names_) {
        output_names_cstr.push_back(name.c_str());
    }
    // 2. 模型推理
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_cstr.data(),  // 用转换后的const char*数组
        &input_tensor,
        1,
        output_names_cstr.data(), // 用转换后的const char*数组
        output_names_cstr.size()
    );
    if (output_tensors.empty()) {
        std::cerr << "[ArmorDetector] 模型推理无输出！" << std::endl;
        return ArmorPose();
    }
    // 3. 解析输出得到6D位姿
    return parseModelOutput(output_tensors[0], undistorted_img, params);
}
