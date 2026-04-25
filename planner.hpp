#pragma once

#include "solve/types.hpp"

#include <memory>

class Planner {
public:
    explicit Planner(const std::string& config_path);
    ~Planner();

    PlanCommand plan(const TargetState& target_state, const GimbalState& gimbal_state);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
