#include "solve/planner.hpp"
#include "common/config.hpp"
#include "common/logger.hpp"
#include "common/types.hpp"

#include <cmath>

class Planner::Impl {
public:
    Impl(const std::string& config_path) {
        Config cfg(config_path);

        auto node = cfg.get_node("planner");
        if (node) {
            fire_threshold_ = node["fire_threshold"].as<double>(0.015);
            horizon_steps_ = node["horizon_steps"].as<int>(100);
        }

        auto solve_node = cfg.get_node("solver");
        if (solve_node) {
            bullet_speed_ = solve_node["bullet_speed"].as<double>(15.0);
        }

        drag_table_ = {
            {0.0, 1.0},
            {5.0, 0.98},
            {10.0, 0.95},
            {15.0, 0.91},
            {20.0, 0.87},
            {25.0, 0.83},
            {30.0, 0.78}
        };
    }

    PlanCommand plan(const TargetState& target, const GimbalState& gimbal) {
        PlanCommand cmd;
        cmd.timestamp = std::chrono::steady_clock::now();

        double dx = target.position.x();
        double dy = target.position.y();
        double dz = target.position.z();
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < 0.01) {
            cmd.control = false;
            cmd.fire = false;
            cmd.yaw = gimbal.yaw;
            cmd.pitch = gimbal.pitch;
            cmd.yaw_vel = 0.0f;
            cmd.pitch_vel = 0.0f;
            cmd.yaw_acc = 0.0f;
            cmd.pitch_acc = 0.0f;
            return cmd;
        }

        double fly_time = bullet_flight_time(distance);

        Eigen::Vector3d predicted_pos = target.position + target.velocity * fly_time;
        double pred_distance = predicted_pos.norm();

        double yaw_target = std::atan2(dy, dx);
        double pitch_target = -std::atan2(dz, std::sqrt(dx * dx + dy * dy));

        double pitch_gravity = ballistic_gravity_compensation(pred_distance);
        double pitch_drag = ballistic_drag_compensation(pred_distance);
        double pitch_total = pitch_target + pitch_gravity + pitch_drag;

        cmd.yaw = static_cast<float>(yaw_target);
        cmd.pitch = static_cast<float>(pitch_total);
        cmd.yaw_vel = 0.0f;
        cmd.pitch_vel = 0.0f;
        cmd.yaw_acc = 0.0f;
        cmd.pitch_acc = 0.0f;

        double gimbal_yaw_diff = normalize_angle_diff(yaw_target - gimbal.yaw);
        double gimbal_pitch_diff = pitch_total - gimbal.pitch;
        double aiming_error = std::sqrt(gimbal_yaw_diff * gimbal_yaw_diff +
                                        gimbal_pitch_diff * gimbal_pitch_diff);

        cmd.fire = (aiming_error < fire_threshold_);
        cmd.control = true;

        return cmd;
    }

private:
    double bullet_flight_time(double distance) const {
        return distance / bullet_speed_;
    }

    double ballistic_gravity_compensation(double distance) const {
        double v = bullet_speed_;
        double t = distance / v;
        double drop = 0.5 * GRAVITY * t * t;
        return std::atan2(drop, distance);
    }

    double ballistic_drag_compensation(double distance) const {
        double drag_factor = interpolate_drag(distance);
        return (1.0 - drag_factor) * 0.01;
    }

    double interpolate_drag(double distance) const {
        if (distance <= drag_table_.front().first) {
            return drag_table_.front().second;
        }
        if (distance >= drag_table_.back().first) {
            return drag_table_.back().second;
        }

        for (size_t i = 0; i < drag_table_.size() - 1; ++i) {
            if (distance >= drag_table_[i].first &&
                distance <= drag_table_[i + 1].first) {
                double t = (distance - drag_table_[i].first) /
                           (drag_table_[i + 1].first - drag_table_[i].first);
                return drag_table_[i].second +
                       t * (drag_table_[i + 1].second - drag_table_[i].second);
            }
        }

        return drag_table_.back().second;
    }

    static double normalize_angle_diff(double diff) {
        while (diff > PI) diff -= 2.0 * PI;
        while (diff < -PI) diff += 2.0 * PI;
        return diff;
    }

    double bullet_speed_ = 15.0;
    double fire_threshold_ = 0.015;
    int horizon_steps_ = 100;
    std::vector<std::pair<double, double>> drag_table_;
};

Planner::Planner(const std::string& config_path)
    : impl_(std::make_unique<Impl>(config_path)) {}

Planner::~Planner() = default;

PlanCommand Planner::plan(const TargetState& target, const GimbalState& gimbal) {
    return impl_->plan(target, gimbal);
}
