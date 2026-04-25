#pragma once

#include "detection/detector.hpp"

#include <yaml-cpp/yaml.h>

struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length;
    float width;
    float angle;
    float area;
    cv::Scalar color;
};

struct TraditionalConfig {
    std::string enemy_color{"RED"};
    int red_threshold{180};
    int blue_threshold{180};
    int gray_threshold{80};

    double min_lightbar_area{20.0};
    double max_lightbar_area{500.0};
    double lightbar_min_aspect{2.0};
    double lightbar_max_aspect{10.0};
    double max_lightbar_angle{45.0};

    double armor_max_angle_diff{15.0};
    double armor_max_height_diff{15.0};
    double armor_min_distance{30.0};
    double armor_max_distance{300.0};
    double armor_max_y_diff{20.0};
};

class TraditionalDetector : public Detector {
public:
    explicit TraditionalDetector(const YAML::Node& config);
    ~TraditionalDetector() override = default;

    std::vector<Armor> detect(const cv::Mat& image) override;
    std::string name() const override;

private:
    bool load_config(const YAML::Node& config);
    cv::Mat preprocess_image(const cv::Mat& image);
    std::vector<LightBar> find_lightbars(const cv::Mat& binary);
    std::vector<std::pair<LightBar, LightBar>> pair_lightbars(
        const std::vector<LightBar>& lightbars);
    Armor build_armor(const LightBar& left, const LightBar& right);

    TraditionalConfig cfg_;
};
