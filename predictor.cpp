#include "solve/predictor.hpp"
#include "common/config.hpp"
#include "common/logger.hpp"
#include "common/types.hpp"
#include "solve/types.hpp"

class Predictor::Impl {
public:
    Impl(const std::string& config_path) {
        Config cfg(config_path);
        auto node = cfg.get_node("predictor");
        if (node) {
            q_pos_ = node["q_pos"].as<double>(0.01);
            q_vel_ = node["q_vel"].as<double>(0.1);
            q_yaw_ = node["q_yaw"].as<double>(0.01);
            r_pos_ = node["r_pos"].as<double>(0.05);
            r_yaw_ = node["r_yaw"].as<double>(0.05);
        }

        x_ = Eigen::VectorXd::Zero(7);
        P_ = Eigen::MatrixXd::Identity(7, 7) * 1.0;
    }

    void init(const Target& target) {
        const auto& pose = target.armor;
        x_(0) = pose.position.x();
        x_(1) = pose.position.y();
        x_(2) = pose.position.z();
        x_(3) = 0.0;
        x_(4) = 0.0;
        x_(5) = 0.0;
        x_(6) = pose.yaw;

        P_ = Eigen::MatrixXd::Identity(7, 7) * 0.5;
        P_(3, 3) = 1.0;
        P_(4, 4) = 1.0;
        P_(5, 5) = 1.0;

        last_timestamp_ = target.last_seen;
        initialized_ = true;

        LOG_INFO("predictor initialized at ({:.2f}, {:.2f}, {:.2f})",
                 x_(0), x_(1), x_(2));
    }

    TargetState predict(const Target& target, double dt) {
        if (!initialized_) {
            init(target);
        }

        auto now = std::chrono::steady_clock::now();
        if (last_timestamp_.time_since_epoch().count() > 0) {
            auto elapsed = std::chrono::duration<double>(now - last_timestamp_).count();
            if (elapsed > 0.0) {
                dt = elapsed;
            }
        }

        Eigen::MatrixXd F = build_F(dt);
        Eigen::MatrixXd Q = build_Q(dt);

        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;

        last_timestamp_ = now;

        return build_target_state();
    }

    TargetState update(const Target& target, double dt) {
        if (!initialized_) {
            init(target);
            return build_target_state();
        }

        if (target.state == TrackerState::LOST) {
            return predict(target, dt);
        }

        auto now = std::chrono::steady_clock::now();
        if (last_timestamp_.time_since_epoch().count() > 0) {
            auto elapsed = std::chrono::duration<double>(now - last_timestamp_).count();
            if (elapsed > 0.0) {
                dt = elapsed;
            }
        }

        Eigen::MatrixXd F = build_F(dt);
        Eigen::MatrixXd Q = build_Q(dt);

        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;

        const auto& pose = target.armor;
        Eigen::VectorXd z(4);
        z(0) = pose.position.x();
        z(1) = pose.position.y();
        z(2) = pose.position.z();
        z(3) = pose.yaw;

        Eigen::MatrixXd H(4, 7);
        H << 1, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 1;

        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
        R(0, 0) = r_pos_;
        R(1, 1) = r_pos_;
        R(2, 2) = r_pos_;
        R(3, 3) = r_yaw_;

        Eigen::MatrixXd K = P_ * H.transpose() *
                            (H * P_ * H.transpose() + R).inverse();

        Eigen::VectorXd y = z - H * x_;
        y(3) = normalize_angle_diff(y(3));

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(7, 7) - K * H) * P_;

        last_timestamp_ = now;

        return build_target_state();
    }

    bool initialized() const {
        return initialized_;
    }

    const Eigen::VectorXd& state() const {
        return x_;
    }

    const Eigen::MatrixXd& covariance() const {
        return P_;
    }

private:
    static Eigen::MatrixXd build_F(double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7, 7);
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;
        F(6, 6) = 1.0;
        return F;
    }

    Eigen::MatrixXd build_Q(double dt) const {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(7, 7);
        Q(0, 0) = q_pos_ * dt;
        Q(1, 1) = q_pos_ * dt;
        Q(2, 2) = q_pos_ * dt;
        Q(3, 3) = q_vel_ * dt;
        Q(4, 4) = q_vel_ * dt;
        Q(5, 5) = q_vel_ * dt;
        Q(6, 6) = q_yaw_ * dt;
        return Q;
    }

    TargetState build_target_state() const {
        TargetState state;
        state.position = Eigen::Vector3d(x_(0), x_(1), x_(2));
        state.velocity = Eigen::Vector3d(x_(3), x_(4), x_(5));
        state.yaw = static_cast<float>(x_(6));
        state.v_yaw = 0.0f;
        state.covariance = P_;
        state.timestamp = last_timestamp_;
        return state;
    }

    static double normalize_angle_diff(double diff) {
        while (diff > PI) diff -= 2.0 * PI;
        while (diff < -PI) diff += 2.0 * PI;
        return diff;
    }

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    std::chrono::steady_clock::time_point last_timestamp_;
    bool initialized_ = false;

    double q_pos_ = 0.01;
    double q_vel_ = 0.1;
    double q_yaw_ = 0.01;
    double r_pos_ = 0.05;
    double r_yaw_ = 0.05;
};

Predictor::Predictor(const std::string& config_path)
    : impl_(std::make_unique<Impl>(config_path)) {}

Predictor::~Predictor() = default;

void Predictor::init(const Target& target) {
    impl_->init(target);
}

TargetState Predictor::predict(const Target& target, double dt) {
    return impl_->predict(target, dt);
}

TargetState Predictor::update(const Target& target, double dt) {
    return impl_->update(target, dt);
}

bool Predictor::initialized() const {
    return impl_->initialized();
}

const Eigen::VectorXd& Predictor::state() const {
    return impl_->state();
}

const Eigen::MatrixXd& Predictor::covariance() const {
    return impl_->covariance();
}
