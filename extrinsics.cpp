#include "solve/extrinsics.hpp"
#include "common/config.hpp"
#include "common/logger.hpp"

Extrinsics::Extrinsics(const std::string& config_path) {
    Config cfg(config_path);
    auto node = cfg.get_node("extrinsics");

    if (!node) {
        LOG_WARN("extrinsics config not found in {}, using defaults (identity)", config_path);
        R_ = Eigen::Matrix3d::Identity();
        t_ = Eigen::Vector3d::Zero();
        return;
    }

    try {
        auto rot_node = node["rotation"];
        if (rot_node && rot_node.IsSequence() && rot_node.size() == 3) {
            for (int i = 0; i < 3; ++i) {
                R_.row(i) = Eigen::Vector3d(rot_node[i][0].as<double>(),
                                            rot_node[i][1].as<double>(),
                                            rot_node[i][2].as<double>());
            }
        } else {
            LOG_WARN("invalid or missing extrinsics.rotation, using identity");
            R_ = Eigen::Matrix3d::Identity();
        }
    } catch (...) {
        LOG_WARN("failed to parse extrinsics.rotation, using identity");
        R_ = Eigen::Matrix3d::Identity();
    }

    try {
        auto trans_node = node["translation"];
        if (trans_node && trans_node.IsSequence() && trans_node.size() == 3) {
            t_ = Eigen::Vector3d(trans_node[0].as<double>(),
                                 trans_node[1].as<double>(),
                                 trans_node[2].as<double>());
        } else {
            LOG_WARN("invalid or missing extrinsics.translation, using zero");
            t_ = Eigen::Vector3d::Zero();
        }
    } catch (...) {
        LOG_WARN("failed to parse extrinsics.translation, using zero");
        t_ = Eigen::Vector3d::Zero();
    }
}

const Eigen::Matrix3d& Extrinsics::rotation() const {
    return R_;
}

const Eigen::Vector3d& Extrinsics::translation() const {
    return t_;
}
