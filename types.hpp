#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

enum class ArmorType : uint8_t {
    SMALL,
    LARGE
};

enum class ArmorName : uint8_t {
    LEFT,
    RIGHT,
    TOP,
    BOTTOM,
    FRONT,
    BACK,
    UNKNOWN
};

inline constexpr const char* ARMOR_NAMES[] = {
    "LEFT", "RIGHT", "TOP", "BOTTOM", "FRONT", "BACK", "UNKNOWN"
};

struct Armor {
    std::array<cv::Point2f, 4> points;
    cv::Point2f center;
    ArmorType type;
    float confidence;
    int class_id;
    float tilt_angle;
};

struct DetectionResult {
    std::vector<Armor> armors;
    int frame_id;
    std::chrono::steady_clock::time_point timestamp;
};
