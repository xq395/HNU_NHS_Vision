#pragma once

#include "detection/types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

enum class TrackerState : uint8_t {
    DETECTING,
    TRACKING,
    LOST
};

struct ArmorPose {
    Eigen::Vector3d position;
    Eigen::Vector3d rotation;
    ArmorType type;
    ArmorName name;
    float yaw;
    float reprojection_error;
};

struct SolveResult {
    std::vector<ArmorPose> armors;
    int frame_id;
    std::chrono::steady_clock::time_point timestamp;
};

struct Target {
    ArmorPose armor;
    TrackerState state;
    int track_id;
    int lost_count;
    std::chrono::steady_clock::time_point last_seen;
};

struct TargetState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    float yaw;
    float v_yaw;
    Eigen::MatrixXd covariance;
    std::chrono::steady_clock::time_point timestamp;
};

struct PlanCommand {
    bool control;
    bool fire;
    float yaw;
    float pitch;
    float yaw_vel;
    float pitch_vel;
    float yaw_acc;
    float pitch_acc;
    std::chrono::steady_clock::time_point timestamp;
};

struct GimbalState {
    float yaw;
    float pitch;
    std::chrono::steady_clock::time_point timestamp;
};
