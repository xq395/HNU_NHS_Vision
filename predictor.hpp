#pragma once

#include "solve/types.hpp"

#include <Eigen/Dense>

#include <memory>
#include <string>

class Predictor {
public:
    explicit Predictor(const std::string& config_path);
    ~Predictor();

    void init(const Target& target);
    TargetState predict(const Target& target, double dt);
    TargetState update(const Target& target, double dt);

    bool initialized() const;
    const Eigen::VectorXd& state() const;
    const Eigen::MatrixXd& covariance() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
