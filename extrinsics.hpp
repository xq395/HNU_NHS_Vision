#pragma once

#include <Eigen/Dense>
#include <string>

class Extrinsics {
public:
    explicit Extrinsics(const std::string& config_path);

    const Eigen::Matrix3d& rotation() const;
    const Eigen::Vector3d& translation() const;

private:
    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;
};
