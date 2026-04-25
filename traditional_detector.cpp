#include "detection/traditional_detector.hpp"
#include "common/logger.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

TraditionalDetector::TraditionalDetector(const YAML::Node& config) {
    if (!load_config(config)) {
        LOG_WARN("TraditionalDetector: using default config");
    }
    LOG_INFO("TraditionalDetector initialized [enemy_color={}]", cfg_.enemy_color);
}

bool TraditionalDetector::load_config(const YAML::Node& config) {
    try {
        cfg_.enemy_color = config["enemy_color"].as<std::string>("RED");
        cfg_.red_threshold = config["red_threshold"].as<int>(40);
        cfg_.blue_threshold = config["blue_threshold"].as<int>(40);
        cfg_.gray_threshold = config["gray_threshold"].as<int>(80);

        cfg_.min_lightbar_area = config["min_lightbar_area"].as<double>(20.0);
        cfg_.max_lightbar_area = config["max_lightbar_area"].as<double>(3000.0);
        cfg_.lightbar_min_aspect = config["lightbar_min_aspect"].as<double>(1.5);
        cfg_.lightbar_max_aspect = config["lightbar_max_aspect"].as<double>(20.0);
        cfg_.max_lightbar_angle = config["max_lightbar_angle"].as<double>(45.0);

        cfg_.armor_max_angle_diff = config["armor_max_angle_diff"].as<double>(25.0);
        cfg_.armor_max_height_diff = config["armor_max_height_diff"].as<double>(15.0);
        cfg_.armor_min_distance = config["armor_min_distance"].as<double>(30.0);
        cfg_.armor_max_distance = config["armor_max_distance"].as<double>(500.0);
        cfg_.armor_max_y_diff = config["armor_max_y_diff"].as<double>(25.0);

        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("TraditionalDetector config error: {}", e.what());
        return false;
    }
}

std::string TraditionalDetector::name() const {
    return "Traditional(" + cfg_.enemy_color + ")";
}

cv::Mat TraditionalDetector::preprocess_image(const cv::Mat& image) {
    cv::Mat gray, binary;

    if (cfg_.enemy_color == "RED") {
        cv::Mat channels[3];
        cv::split(image, channels);
        cv::Mat sub = channels[2] - channels[0];
        cv::threshold(sub, binary, cfg_.red_threshold, 255, cv::THRESH_BINARY);
    } else {
        cv::Mat channels[3];
        cv::split(image, channels);
        cv::Mat sub = channels[0] - channels[2];
        cv::threshold(sub, binary, cfg_.blue_threshold, 255, cv::THRESH_BINARY);
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    return binary;
}

std::vector<LightBar> TraditionalDetector::find_lightbars(const cv::Mat& binary) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> lightbars;
    lightbars.reserve(contours.size());

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < cfg_.min_lightbar_area || area > cfg_.max_lightbar_area) continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);

        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) {
            std::swap(width, height);
        }

        double aspect = height / (width + 1e-6);
        if (aspect < cfg_.lightbar_min_aspect || aspect > cfg_.lightbar_max_aspect) continue;

        float angle = rect.angle;
        if (rect.size.width < rect.size.height) {
            angle = 90.0f + angle;
        }
        if (std::abs(angle) > cfg_.max_lightbar_angle &&
            std::abs(angle - 180.0f) > cfg_.max_lightbar_angle) continue;

        LightBar lb;
        lb.rect = rect;
        lb.center = rect.center;
        lb.length = height;
        lb.width = width;
        lb.angle = angle;
        lb.area = static_cast<float>(area);
        lightbars.push_back(lb);
    }

    return lightbars;
}

std::vector<std::pair<LightBar, LightBar>> TraditionalDetector::pair_lightbars(
    const std::vector<LightBar>& lightbars) {

    std::vector<std::pair<LightBar, LightBar>> pairs;
    pairs.reserve(lightbars.size());

    for (size_t i = 0; i < lightbars.size(); ++i) {
        for (size_t j = i + 1; j < lightbars.size(); ++j) {
            const auto& l1 = lightbars[i];
            const auto& l2 = lightbars[j];

            float angle_diff = std::abs(l1.angle - l2.angle);
            if (angle_diff > cfg_.armor_max_angle_diff &&
                angle_diff < 180.0f - cfg_.armor_max_angle_diff) continue;

            float height_diff = std::abs(l1.length - l2.length);
            if (height_diff > cfg_.armor_max_height_diff) continue;

            float dx = l2.center.x - l1.center.x;
            float dy = l2.center.y - l1.center.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < cfg_.armor_min_distance ||
                distance > cfg_.armor_max_distance) continue;

            if (std::abs(dy) > cfg_.armor_max_y_diff) continue;

            float avg_height = (l1.length + l2.length) / 2.0f;
            float armor_ratio = distance / (avg_height + 1e-6f);
            if (armor_ratio > 4.5f) continue;

            if (l1.center.x < l2.center.x) {
                pairs.emplace_back(l1, l2);
            } else {
                pairs.emplace_back(l2, l1);
            }
        }
    }

    return pairs;
}

Armor TraditionalDetector::build_armor(const LightBar& left, const LightBar& right) {
    cv::Point2f left_points[4], right_points[4];
    left.rect.points(left_points);
    right.rect.points(right_points);

    auto top_point = [](const cv::Point2f pts[4]) -> cv::Point2f {
        cv::Point2f top = pts[0];
        for (int i = 1; i < 4; ++i) {
            if (pts[i].y < top.y) top = pts[i];
        }
        return top;
    };

    auto bottom_point = [](const cv::Point2f pts[4]) -> cv::Point2f {
        cv::Point2f bottom = pts[0];
        for (int i = 1; i < 4; ++i) {
            if (pts[i].y > bottom.y) bottom = pts[i];
        }
        return bottom;
    };

    cv::Point2f lt = top_point(left_points);
    cv::Point2f lb = bottom_point(left_points);
    cv::Point2f rt = top_point(right_points);
    cv::Point2f rb = bottom_point(right_points);

    float tilt_angle = std::atan2(rt.y - lt.y, rt.x - lt.x);

    Armor armor;
    armor.points = {lt, rt, rb, lb};

    armor.center = (lt + rt + rb + lb) * 0.25f;

    float width = cv::norm(lt - rt);
    float height = cv::norm(lt - lb);
    armor.type = (width / height > 3.2f) ? ArmorType::LARGE : ArmorType::SMALL;

    armor.confidence = 1.0f;
    armor.class_id = 0;
    armor.tilt_angle = tilt_angle;

    return armor;
}

std::vector<Armor> TraditionalDetector::detect(const cv::Mat& image) {
    cv::Mat binary = preprocess_image(image);
    auto lightbars = find_lightbars(binary);

    if (lightbars.empty()) return {};

    auto pairs = pair_lightbars(lightbars);

    std::vector<Armor> armors;
    armors.reserve(pairs.size());
    for (const auto& [left, right] : pairs) {
        armors.push_back(build_armor(left, right));
    }

    return armors;
}
