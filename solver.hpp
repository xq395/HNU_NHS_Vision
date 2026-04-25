#pragma once

#include "detection/types.hpp"
#include "solve/types.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <memory>
#include <string>

class Solver {
public:
    explicit Solver(const std::string& config_path);
    ~Solver();

    SolveResult solve(const DetectionResult& det,
                      const cv::Mat& camera_matrix,
                      const cv::Mat& dist_coeffs,
                      const Eigen::Quaterniond& imu_orientation);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
