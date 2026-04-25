#include "detection/detector.hpp"
#include "detection/traditional_detector.hpp"

#ifdef HAS_ONNX_RUNTIME
#include "detection/yolo_detector.hpp"
#endif

#include "common/logger.hpp"
#include "common/config.hpp"

#include <yaml-cpp/yaml.h>

std::unique_ptr<Detector> Detector::create(const YAML::Node& config) {
    auto mode = config["mode"].as<std::string>("yolo26");

#ifdef HAS_ONNX_RUNTIME
    if (mode == "yolov8" || mode == "yolo11" || mode == "yolo26" || mode == "yolov5") {
        LOG_INFO("Creating YOLO detector, mode: {}", mode);
        auto det = std::make_unique<YOLODetector>(config);
        if (!det->valid()) {
            LOG_ERROR("YOLO detector initialization failed!");
            return nullptr;
        }
        return det;
    }
#else
    if (mode == "yolov8" || mode == "yolo11" || mode == "yolo26" || mode == "yolov5") {
        LOG_WARN("YOLO mode '{}' requested but ONNX Runtime is not available. "
                 "Please install ONNX Runtime C++ library to use YOLO.", mode);
        return nullptr;
    }
#endif

    if (mode == "traditional") {
        LOG_INFO("Creating traditional detector");
        return std::make_unique<TraditionalDetector>(config);
    }

    LOG_WARN("Unknown detection mode '{}', falling back to traditional", mode);
    return std::make_unique<TraditionalDetector>(config);
}

std::unique_ptr<Detector> Detector::create(const std::string& config_path) {
    Config cfg(config_path);
    auto node = cfg.get_node("detection");
    return create(node);
}
