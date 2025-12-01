#ifndef IMG_TOOLS_HPP
#define IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>

// 图像缩放（适配ONNX模型输入尺寸）
inline cv::Mat resizeImage(const cv::Mat& src, int target_w = 640, int target_h = 480) {
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(target_w, target_h), 0, 0, cv::INTER_LINEAR);
    return dst;
}

// 绘制装甲板旋转矩形（验收要求：框出识别到的装甲板）
inline void drawArmorRect(cv::Mat& img, const cv::RotatedRect& rrect, 
                          const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 2) {
    if (img.empty() || rrect.size.width <= 0 || rrect.size.height <= 0) return;
    cv::Point2f vertices[4];
    rrect.points(vertices);
    for (int i = 0; i < 4; ++i) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
    }
    // 绘制装甲板中心
    cv::circle(img, rrect.center, 4, cv::Scalar(0, 0, 255), -1);
}

// 在图像上绘制文本信息（如6D位姿、yaw/pitch角度）
inline void drawImageText(cv::Mat& img, const std::string& text, 
                          const cv::Point& pos, const cv::Scalar& color = cv::Scalar(255, 0, 0)) {
    if (img.empty()) return;
    cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
}

#endif // IMG_TOOLS_HPP