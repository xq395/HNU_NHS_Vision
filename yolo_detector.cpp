#include "detection/yolo_detector.hpp"
#include "common/logger.hpp"

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

class YOLODetector::Impl {
public:
    Ort::Env env{Ort::Env(ORT_LOGGING_LEVEL_WARNING, "yolo_detector")};
    Ort::SessionOptions session_options;
    Ort::Session session{nullptr};
    Ort::MemoryInfo memory_info{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<int64_t> input_dims;
    int64_t input_tensor_size{1};

    Impl() {
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_options.SetIntraOpNumThreads(4);
    }
};

YOLODetector::YOLODetector(const YAML::Node& config)
    : impl_(std::make_unique<Impl>())
    , preprocessor_(640, 640) {

    if (!load_model_config(config)) {
        LOG_ERROR("YOLODetector: failed to load model config");
        return;
    }

    preprocessor_ = Preprocessor(cfg_.input_width, cfg_.input_height);

    if (!load_model(cfg_.model_path)) {
        LOG_ERROR("YOLODetector: failed to load model from {}", cfg_.model_path);
        return;
    }

    valid_ = true;
    LOG_INFO("YOLODetector initialized [mode={}, model={}, input={}x{}, conf={}, nms={}]",
             cfg_.mode, cfg_.model_path, cfg_.input_width, cfg_.input_height,
             cfg_.conf_threshold, cfg_.nms_threshold);
}

YOLODetector::~YOLODetector() = default;

bool YOLODetector::load_model_config(const YAML::Node& config) {
    try {
        cfg_.model_path = config["model_path"].as<std::string>("assets/models/yolo26.onnx");
        cfg_.input_width = config["input_width"].as<int>(640);
        cfg_.input_height = config["input_height"].as<int>(640);
        cfg_.conf_threshold = config["conf_threshold"].as<float>(0.5f);
        cfg_.nms_threshold = config["nms_threshold"].as<float>(0.3f);
        cfg_.mode = config["mode"].as<std::string>("yolo26");
        cfg_.num_classes = config["num_classes"].as<int>(1);
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("YOLODetector config error: {}", e.what());
        return false;
    }
}

bool YOLODetector::load_model(const std::string& model_path) {
    try {
        impl_->session = Ort::Session(impl_->env, model_path.c_str(), impl_->session_options);

        Ort::AllocatorWithDefaultOptions allocator;

        size_t num_inputs = impl_->session.GetInputCount();
        size_t num_outputs = impl_->session.GetOutputCount();

        impl_->input_names.resize(num_inputs);
        impl_->output_names.resize(num_outputs);

        for (size_t i = 0; i < num_inputs; ++i) {
            auto name = impl_->session.GetInputNameAllocated(i, allocator);
            impl_->input_names[i] = strdup(name.get());
        }
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name = impl_->session.GetOutputNameAllocated(i, allocator);
            impl_->output_names[i] = strdup(name.get());
        }

        auto type_info = impl_->session.GetInputTypeInfo(0);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        impl_->input_dims = tensor_info.GetShape();

        impl_->input_tensor_size = 1;
        for (auto dim : impl_->input_dims) {
            if (dim > 0) impl_->input_tensor_size *= dim;
        }

        LOG_INFO("Model loaded: {} inputs, {} outputs, tensor size={}",
                 num_inputs, num_outputs, impl_->input_tensor_size);

        return true;
    } catch (const Ort::Exception& e) {
        LOG_ERROR("ONNX Runtime error loading model: {}", e.what());
        return false;
    }
}

namespace {
uint16_t float32_to_float16(float f) {
    uint32_t fi;
    std::memcpy(&fi, &f, sizeof(fi));

    uint16_t sign = static_cast<uint16_t>((fi >> 16) & 0x8000);
    int32_t exponent = static_cast<int32_t>(((fi >> 23) & 0xff) - 127 + 15);
    uint32_t mantissa = fi & 0x007fffff;

    if (exponent <= 0) {
        if (exponent < -10) return sign;
        mantissa = (mantissa | 0x00800000) >> (1 - exponent);
        return static_cast<uint16_t>(sign | (mantissa >> 13));
    }
    if (exponent > 30) {
        return static_cast<uint16_t>(sign | 0x7c00 | (mantissa >> 13));
    }
    return static_cast<uint16_t>(sign | static_cast<uint16_t>(exponent << 10) | (mantissa >> 13));
}
}

std::vector<float> YOLODetector::preprocess(const cv::Mat& image) {
    auto prep_result = preprocessor_.letterbox(image);

    cv::Mat blob = cv::dnn::blobFromImage(prep_result.blob);

    std::vector<float> input_tensor_values;
    input_tensor_values.assign(blob.begin<float>(), blob.end<float>());
    return input_tensor_values;
}

std::vector<uint16_t> YOLODetector::preprocess_yolov5(const cv::Mat& image, PreprocessResult& prep_result) {
    prep_result.original_size = image.size();

    float scale = std::min(
        static_cast<float>(cfg_.input_width) / image.cols,
        static_cast<float>(cfg_.input_height) / image.rows);
    prep_result.scale = scale;

    int new_w = static_cast<int>(image.cols * scale);
    int new_h = static_cast<int>(image.rows * scale);

    cv::Mat canvas(cfg_.input_height, cfg_.input_width, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(new_w, new_h));
    resized.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));

    prep_result.pad_w = 0;
    prep_result.pad_h = 0;

    cv::Mat float_img;
    canvas.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    cv::Mat rgb_img;
    cv::cvtColor(float_img, rgb_img, cv::COLOR_BGR2RGB);

    size_t total_pixels = static_cast<size_t>(cfg_.input_height) * cfg_.input_width;
    std::vector<uint16_t> fp16_data(3 * total_pixels);
    const float* data = rgb_img.ptr<float>();
    for (int c = 0; c < 3; ++c) {
        for (size_t i = 0; i < total_pixels; ++i) {
            fp16_data[c * total_pixels + i] = float32_to_float16(data[i * 3 + c]);
        }
    }
    return fp16_data;
}

std::vector<DetectedBox> YOLODetector::postprocess_yolov8(
    const float* raw_output, int num_boxes, int stride) {

    std::vector<DetectedBox> candidates;
    candidates.reserve(num_boxes);

    for (int i = 0; i < num_boxes; ++i) {
        const float* box_data = raw_output + i * stride;

        float cx = box_data[0];
        float cy = box_data[1];
        float w = box_data[2];
        float h = box_data[3];

        float max_class_score = 0.0f;
        int best_class_id = -1;
        for (int j = 0; j < cfg_.num_classes; ++j) {
            float score = box_data[4 + j];
            if (score > max_class_score) {
                max_class_score = score;
                best_class_id = j;
            }
        }

        if (max_class_score < cfg_.conf_threshold) continue;

        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;
        float x2 = cx + w / 2.0f;
        float y2 = cy + h / 2.0f;

        DetectedBox box;
        box.cx = cx;
        box.cy = cy;
        box.w = w;
        box.h = h;
        box.confidence = max_class_score;
        box.class_id = best_class_id;
        box.four_points = {
            {x1, y1},
            {x2, y1},
            {x2, y2},
            {x1, y2}
        };

        candidates.push_back(box);
    }

    return nms(candidates);
}

std::vector<DetectedBox> YOLODetector::postprocess_yolo26(
    const float* raw_output, int num_boxes, int stride) {

    std::vector<DetectedBox> candidates;
    candidates.reserve(num_boxes);

    for (int i = 0; i < num_boxes; ++i) {
        const float* box_data = raw_output + i * stride;

        float conf = box_data[8];
        if (conf < cfg_.conf_threshold) continue;

        int best_class_id = 0;
        float max_class_score = conf;
        if (cfg_.num_classes > 1) {
            for (int j = 0; j < cfg_.num_classes; ++j) {
                float score = box_data[9 + j];
                if (score > max_class_score) {
                    max_class_score = score;
                    best_class_id = j;
                }
            }
        }

        float x1 = box_data[0], y1 = box_data[1];
        float x2 = box_data[2], y2 = box_data[3];
        float x3 = box_data[4], y3 = box_data[5];
        float x4 = box_data[6], y4 = box_data[7];

        float cx = (x1 + x2 + x3 + x4) / 4.0f;
        float cy = (y1 + y2 + y3 + y4) / 4.0f;
        float w = std::max(x2 - x1, x3 - x4);
        float h = std::max(y3 - y1, y4 - y2);

        DetectedBox box;
        box.cx = cx;
        box.cy = cy;
        box.w = w;
        box.h = h;
        box.confidence = max_class_score;
        box.class_id = best_class_id;
        box.four_points = {{x1, y1}, {x2, y2}, {x3, y3}, {x4, y4}};

        candidates.push_back(box);
    }

    return nms(candidates);
}

std::vector<DetectedBox> YOLODetector::postprocess_yolov5(
    const float* raw_output, int num_boxes, int stride) {

    std::vector<DetectedBox> candidates;
    candidates.reserve(num_boxes);

    for (int i = 0; i < num_boxes; ++i) {
        const float* box_data = raw_output + i * stride;

        float objectness = 1.0f / (1.0f + std::exp(-box_data[cfg_.obj_channel]));
        if (objectness < cfg_.conf_threshold) continue;

        float best_color_score = 0.0f;
        int best_color_id = -1;
        for (int c = 0; c < 4; ++c) {
            float score = 1.0f / (1.0f + std::exp(-box_data[cfg_.color_start + c]));
            if (score > best_color_score) {
                best_color_score = score;
                best_color_id = c;
            }
        }

        float best_digit_score = 0.0f;
        int best_digit_id = -1;
        for (int d = 0; d < 9; ++d) {
            float score = 1.0f / (1.0f + std::exp(-box_data[cfg_.color_start + 4 + d]));
            if (score > best_digit_score) {
                best_digit_score = score;
                best_digit_id = d;
            }
        }

        float final_conf = objectness;

        int class_id = best_digit_id;

        std::vector<cv::Point2f> pts(4);
        pts[0] = cv::Point2f(box_data[cfg_.pt0_x], box_data[cfg_.pt0_y]);
        pts[1] = cv::Point2f(box_data[cfg_.pt1_x], box_data[cfg_.pt1_y]);
        pts[2] = cv::Point2f(box_data[cfg_.pt2_x], box_data[cfg_.pt2_y]);
        pts[3] = cv::Point2f(box_data[cfg_.pt3_x], box_data[cfg_.pt3_y]);

        float cx = 0, cy = 0;
        for (const auto& p : pts) { cx += p.x; cy += p.y; }
        cx /= 4.0f; cy /= 4.0f;

        float x1 = std::min({pts[0].x, pts[1].x, pts[2].x, pts[3].x});
        float y1 = std::min({pts[0].y, pts[1].y, pts[2].y, pts[3].y});
        float x2 = std::max({pts[0].x, pts[1].x, pts[2].x, pts[3].x});
        float y2 = std::max({pts[0].y, pts[1].y, pts[2].y, pts[3].y});

        DetectedBox box;
        box.cx = cx;
        box.cy = cy;
        box.w = x2 - x1;
        box.h = y2 - y1;
        box.confidence = final_conf;
        box.class_id = class_id;
        box.four_points = pts;

        candidates.push_back(box);
    }

    return nms(candidates);
}

std::vector<DetectedBox> YOLODetector::nms(const std::vector<DetectedBox>& boxes) {
    if (boxes.empty()) return {};

    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return boxes[a].confidence > boxes[b].confidence;
    });

    std::vector<bool> suppressed(boxes.size(), false);
    std::vector<DetectedBox> result;

    for (size_t i = 0; i < indices.size(); ++i) {
        if (suppressed[indices[i]]) continue;

        const auto& box_a = boxes[indices[i]];
        result.push_back(box_a);

        float area_a = box_a.w * box_a.h;

        for (size_t j = i + 1; j < indices.size(); ++j) {
            if (suppressed[indices[j]]) continue;

            const auto& box_b = boxes[indices[j]];

            float inter_x1 = std::max(box_a.cx - box_a.w / 2, box_b.cx - box_b.w / 2);
            float inter_y1 = std::max(box_a.cy - box_a.h / 2, box_b.cy - box_b.h / 2);
            float inter_x2 = std::min(box_a.cx + box_a.w / 2, box_b.cx + box_b.w / 2);
            float inter_y2 = std::min(box_a.cy + box_a.h / 2, box_b.cy + box_b.h / 2);

            float inter_w = std::max(0.0f, inter_x2 - inter_x1);
            float inter_h = std::max(0.0f, inter_y2 - inter_y1);
            float inter_area = inter_w * inter_h;

            float area_b = box_b.w * box_b.h;
            float iou = inter_area / (area_a + area_b - inter_area);

            if (iou > cfg_.nms_threshold) {
                suppressed[indices[j]] = true;
            }
        }
    }

    return result;
}

void YOLODetector::normalize_armor_points(std::vector<cv::Point2f>& points) {
    if (points.size() != 4) return;

    cv::Point2f center(0, 0);
    for (const auto& p : points) center += p;
    center *= 0.25f;

    auto angle = [&](const cv::Point2f& p) -> float {
        return std::atan2(p.y - center.y, p.x - center.x);
    };

    std::sort(points.begin(), points.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
        return angle(a) < angle(b);
    });

    std::vector<cv::Point2f> ordered(4);
    ordered[0] = points[0];
    ordered[1] = points[1];
    ordered[2] = points[3];
    ordered[3] = points[2];

    points = ordered;
}

std::string YOLODetector::name() const {
    return "YOLO(" + cfg_.mode + ")";
}

std::vector<Armor> YOLODetector::detect(const cv::Mat& image) {
    if (!valid_ || !impl_ || !impl_->session) {
        LOG_ERROR("YOLODetector not properly initialized");
        return {};
    }

    PreprocessResult prep_result;
    std::vector<Ort::Value> input_tensors;
    std::vector<uint16_t> fp16_data;

    Ort::AllocatorWithDefaultOptions allocator;

    if (cfg_.mode == "yolov5") {
        fp16_data = preprocess_yolov5(image, prep_result);
        input_tensors.push_back(Ort::Value::CreateTensor(
            impl_->memory_info, static_cast<void*>(fp16_data.data()),
            fp16_data.size() * sizeof(uint16_t),
            impl_->input_dims.data(), impl_->input_dims.size(),
            ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16));
    } else {
        auto input_values = preprocess(image);

        prep_result.original_size = image.size();
        prep_result.scale = std::min(
            static_cast<float>(cfg_.input_width) / image.cols,
            static_cast<float>(cfg_.input_height) / image.rows);
        prep_result.pad_w = static_cast<int>((cfg_.input_width - image.cols * prep_result.scale) / 2);
        prep_result.pad_h = static_cast<int>((cfg_.input_height - image.rows * prep_result.scale) / 2);

        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            impl_->memory_info, input_values.data(), input_values.size(),
            impl_->input_dims.data(), impl_->input_dims.size()));
    }

    std::vector<Ort::Value> output_tensors;
    try {
        output_tensors = impl_->session.Run(Ort::RunOptions{nullptr},
                                            impl_->input_names.data(), input_tensors.data(),
                                            static_cast<int>(impl_->input_names.size()),
                                            impl_->output_names.data(),
                                            static_cast<int>(impl_->output_names.size()));
    } catch (const Ort::Exception& e) {
        LOG_ERROR("ONNX Runtime inference error: {}", e.what());
        return {};
    }

    if (output_tensors.empty()) return {};

    float* raw_output = output_tensors[0].GetTensorMutableData<float>();
    auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    int num_boxes = 1;
    int stride = 1;
    if (output_shape.size() == 3) {
        num_boxes = static_cast<int>(output_shape[1]);
        stride = static_cast<int>(output_shape[2]);
    } else if (output_shape.size() == 2) {
        num_boxes = static_cast<int>(output_shape[0]);
        stride = static_cast<int>(output_shape[1]);
    }

    std::vector<DetectedBox> detected;
    if (cfg_.mode == "yolo26") {
        detected = postprocess_yolo26(raw_output, num_boxes, stride);
    } else if (cfg_.mode == "yolov5") {
        detected = postprocess_yolov5(raw_output, num_boxes, stride);
    } else {
        detected = postprocess_yolov8(raw_output, num_boxes, stride);
    }

    std::vector<Armor> armors;
    armors.reserve(detected.size());

    for (auto& box : detected) {
        std::vector<cv::Point2f> scaled = Preprocessor::scale_coords(box.four_points, prep_result);

        normalize_armor_points(scaled);

        Armor armor;
        for (int i = 0; i < 4; ++i) {
            armor.points[i] = scaled[i];
        }
        armor.center = {box.cx, box.cy};
        armor.center = Preprocessor::scale_coords({armor.center}, prep_result)[0];

        float width = cv::norm(scaled[0] - scaled[1]);
        float height = cv::norm(scaled[1] - scaled[2]);
        armor.type = (width / height > 3.2f) ? ArmorType::LARGE : ArmorType::SMALL;

        armor.confidence = box.confidence;
        armor.class_id = box.class_id;
        armor.tilt_angle = std::atan2(scaled[1].y - scaled[0].y, scaled[1].x - scaled[0].x);

        armors.push_back(armor);
    }

    return armors;
}
