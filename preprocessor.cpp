#include "detection/preprocessor.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

Preprocessor::Preprocessor(int input_width, int input_height)
    : input_width_(input_width)
    , input_height_(input_height) {
}

PreprocessResult Preprocessor::letterbox(const cv::Mat& image) {
    PreprocessResult result;
    result.original_size = image.size();

    float scale = std::min(
        static_cast<float>(input_width_) / image.cols,
        static_cast<float>(input_height_) / image.rows);
    result.scale = scale;

    int new_w = static_cast<int>(image.cols * scale);
    int new_h = static_cast<int>(image.rows * scale);
    result.pad_w = (input_width_ - new_w) / 2;
    result.pad_h = (input_height_ - new_h) / 2;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));

    cv::Mat canvas(input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(result.pad_w, result.pad_h, new_w, new_h)));

    canvas.convertTo(result.blob, CV_32FC3, 1.0f / 255.0f);
    return result;
}

std::vector<cv::Point2f> Preprocessor::scale_coords(
    const std::vector<cv::Point2f>& points,
    const PreprocessResult& prep) {

    std::vector<cv::Point2f> out;
    out.reserve(points.size());
    for (const auto& pt : points) {
        float x = (pt.x - prep.pad_w) / prep.scale;
        float y = (pt.y - prep.pad_h) / prep.scale;
        x = std::clamp(x, 0.0f, static_cast<float>(prep.original_size.width - 1));
        y = std::clamp(y, 0.0f, static_cast<float>(prep.original_size.height - 1));
        out.emplace_back(x, y);
    }
    return out;
}
