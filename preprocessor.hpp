#pragma once

#include <cmath>
#include <opencv2/core.hpp>

struct PreprocessResult {
    cv::Mat blob;
    float scale;
    int pad_w;
    int pad_h;
    cv::Size original_size;
};

class Preprocessor {
public:
    explicit Preprocessor(int input_width, int input_height);

    PreprocessResult letterbox(const cv::Mat& image);

    static std::vector<cv::Point2f> scale_coords(
        const std::vector<cv::Point2f>& points,
        const PreprocessResult& prep);

private:
    int input_width_;
    int input_height_;
};
