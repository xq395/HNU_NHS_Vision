#pragma once

#include "solve/types.hpp"

#include <memory>
#include <string>
#include <vector>

class Tracker {
public:
    explicit Tracker(const std::string& config_path);
    ~Tracker();

    Target update(const std::vector<ArmorPose>& armors, int frame_id);

    TrackerState state() const;

    const Target& tracked_target() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
