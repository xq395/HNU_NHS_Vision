#pragma once

#include "detection/types.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

class Detector {
public:
    virtual ~Detector() = default;

    virtual std::vector<Armor> detect(const cv::Mat& image) = 0;

    virtual std::string name() const = 0;

    static std::unique_ptr<Detector> create(const YAML::Node& config);

    static std::unique_ptr<Detector> create(const std::string& config_path);
};
