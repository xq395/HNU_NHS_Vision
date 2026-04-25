#pragma once

#include "detection/detector.hpp"
#include "detection/preprocessor.hpp"

#include <yaml-cpp/yaml.h>

struct YOLOModelConfig {
    std::string model_path;
    int input_width{640};
    int input_height{640};
    float conf_threshold{0.5f};
    float nms_threshold{0.3f};
    std::string mode{"yolo26"};
    int num_classes{1};

    int obj_channel{8};
    int color_start{9};
    int num_digits{9};
    int pt0_x{0}, pt0_y{1};
    int pt1_x{6}, pt1_y{7};
    int pt2_x{4}, pt2_y{5};
    int pt3_x{2}, pt3_y{3};
};

struct DetectedBox {
    float cx, cy, w, h;
    float confidence;
    int class_id;
    std::vector<cv::Point2f> four_points;
};

class YOLODetector : public Detector {
public:
    explicit YOLODetector(const YAML::Node& config);
    ~YOLODetector() override;

    std::vector<Armor> detect(const cv::Mat& image) override;
    std::string name() const override;

    bool valid() const { return valid_; }

private:
    bool load_model(const std::string& model_path);
    std::vector<float> preprocess(const cv::Mat& image);
    std::vector<uint16_t> preprocess_yolov5(const cv::Mat& image, PreprocessResult& prep_result);
    std::vector<DetectedBox> postprocess_yolov8(const float* raw_output,
                                                 int num_boxes, int stride);
    std::vector<DetectedBox> postprocess_yolo26(const float* raw_output,
                                                 int num_boxes, int stride);
    std::vector<DetectedBox> postprocess_yolov5(const float* raw_output,
                                                 int num_boxes, int stride);
    std::vector<DetectedBox> nms(const std::vector<DetectedBox>& boxes);
    bool load_model_config(const YAML::Node& config);

    void normalize_armor_points(std::vector<cv::Point2f>& points);

    YOLOModelConfig cfg_;
    Preprocessor preprocessor_;
    bool valid_{false};

    class Impl;
    std::unique_ptr<Impl> impl_;
};
