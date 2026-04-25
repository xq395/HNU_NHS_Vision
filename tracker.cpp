#include "solve/tracker.hpp"
#include "common/config.hpp"
#include "common/logger.hpp"
#include "common/types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

struct Track {
    Target target;
    int detect_count = 0;
    int lost_count = 0;
};

static double compute_iou_2d(const ArmorPose& a, const ArmorPose& b) {
    double ax = a.position.x();
    double ay = a.position.y();
    double bx = b.position.x();
    double by = b.position.y();
    double aw = 0.2;
    double ah = 0.1;
    double bw = 0.2;
    double bh = 0.1;

    double x1 = std::max(ax - aw / 2.0, bx - bw / 2.0);
    double y1 = std::max(ay - ah / 2.0, by - bh / 2.0);
    double x2 = std::min(ax + aw / 2.0, bx + bw / 2.0);
    double y2 = std::min(ay + ah / 2.0, by + bh / 2.0);

    double inter_area = std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
    double area_a = aw * ah;
    double area_b = bw * bh;
    double union_area = area_a + area_b - inter_area;

    return (union_area > 0.0) ? (inter_area / union_area) : 0.0;
}

static double compute_association_cost(const ArmorPose& track_pose, const ArmorPose& det_pose) {
    double iou = compute_iou_2d(track_pose, det_pose);
    double iou_cost = (iou > 0.0) ? (1.0 - iou) : 5.0;

    Eigen::Vector3d diff = det_pose.position - track_pose.position;
    double dist = diff.norm();

    double yaw_diff = std::abs(det_pose.yaw - track_pose.yaw);
    while (yaw_diff > PI) yaw_diff -= 2.0 * PI;
    while (yaw_diff < -PI) yaw_diff += 2.0 * PI;

    return iou_cost * 0.4 + dist * 0.002 + yaw_diff * 0.1;
}

class Tracker::Impl {
public:
    Impl(const std::string& config_path) {
        Config cfg(config_path);
        auto node = cfg.get_node("tracker");
        if (node) {
            min_detect_count_ = node["min_detect_count"].as<int>(3);
            max_lost_count_ = node["max_lost_count"].as<int>(10);
            max_tracks_ = node["max_tracks"].as<int>(6);
        }
    }

    Target update(const std::vector<ArmorPose>& armors, int frame_id) {
        auto now = std::chrono::steady_clock::now();
        update_tracks_lost(now);

        if (armors.empty()) {
            return best_track_target();
        }

        associate_and_update(armors, now);

        cleanup_dead_tracks();

        return best_track_target();
    }

    TrackerState state() const {
        return state_;
    }

    const Target& tracked_target() const {
        return best_track_;
    }

private:
    void update_tracks_lost(const std::chrono::steady_clock::time_point& now) {
        if (tracks_.empty()) {
            return;
        }

        for (auto& track : tracks_) {
            if (track.target.state == TrackerState::TRACKING) {
                track.lost_count++;
                if (track.lost_count >= max_lost_count_) {
                    track.target.state = TrackerState::LOST;
                    LOG_DEBUG("track {} lost after {} frames",
                              track.target.track_id, track.lost_count);
                }
            }
        }

        update_best_track();
    }

    void associate_and_update(
        const std::vector<ArmorPose>& armors,
        const std::chrono::steady_clock::time_point& now)
    {
        std::vector<bool> track_matched(tracks_.size(), false);
        std::vector<bool> det_matched(armors.size(), false);

        for (size_t ti = 0; ti < tracks_.size(); ++ti) {
            if (tracks_[ti].target.state == TrackerState::LOST) {
                continue;
            }

            double best_cost = std::numeric_limits<double>::max();
            int best_di = -1;

            for (size_t di = 0; di < armors.size(); ++di) {
                if (det_matched[di]) {
                    continue;
                }

                double cost = compute_association_cost(
                    tracks_[ti].target.armor, armors[di]);

                if (cost < best_cost) {
                    best_cost = cost;
                    best_di = static_cast<int>(di);
                }
            }

            if (best_di >= 0 && best_cost < 10.0) {
                track_matched[ti] = true;
                det_matched[best_di] = true;
                update_track(tracks_[ti], armors[best_di], now);
            }
        }

        for (size_t di = 0; di < armors.size(); ++di) {
            if (!det_matched[di]) {
                create_new_track(armors[di], now);
            }
        }
    }

    void update_track(Track& track, const ArmorPose& armor,
                      const std::chrono::steady_clock::time_point& now) {
        track.target.armor = armor;
        track.target.last_seen = now;
        track.lost_count = 0;

        switch (track.target.state) {
            case TrackerState::DETECTING:
                track.detect_count++;
                if (track.detect_count >= min_detect_count_) {
                    track.target.state = TrackerState::TRACKING;
                    LOG_INFO("track {} confirmed TRACKING", track.target.track_id);
                }
                break;

            case TrackerState::TRACKING:
                break;

            default:
                break;
        }

        update_best_track();
    }

    void create_new_track(const ArmorPose& armor,
                          const std::chrono::steady_clock::time_point& now) {
        if (tracks_.size() >= static_cast<size_t>(max_tracks_)) {
            tracks_.erase(
                std::remove_if(tracks_.begin(), tracks_.end(),
                    [](const Track& t) {
                        return t.target.state == TrackerState::LOST;
                    }),
                tracks_.end());
        }

        if (tracks_.size() >= static_cast<size_t>(max_tracks_)) {
            return;
        }

        Track new_track;
        new_track.target.armor = armor;
        new_track.target.state = TrackerState::DETECTING;
        new_track.target.track_id = next_id_++;
        new_track.target.lost_count = 0;
        new_track.target.last_seen = now;
        new_track.detect_count = 1;
        new_track.lost_count = 0;

        tracks_.push_back(new_track);
        LOG_INFO("new track {} created (DETECTING)", new_track.target.track_id);

        update_best_track();
    }

    void cleanup_dead_tracks() {
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                [this](const Track& t) {
                    return t.target.state == TrackerState::LOST &&
                           t.lost_count >= max_lost_count_ * 2;
                }),
            tracks_.end());
    }

    void update_best_track() {
        int best_id = -1;

        for (const auto& track : tracks_) {
            if (track.target.state == TrackerState::TRACKING) {
                best_id = track.target.track_id;
                if (state_ != TrackerState::TRACKING) {
                    state_ = TrackerState::TRACKING;
                }
                break;
            }
        }

        if (best_id < 0) {
            for (const auto& track : tracks_) {
                if (track.target.state == TrackerState::DETECTING) {
                    best_id = track.target.track_id;
                    if (state_ != TrackerState::DETECTING) {
                        state_ = TrackerState::DETECTING;
                    }
                    break;
                }
            }
        }

        if (best_id >= 0) {
            auto it = std::find_if(tracks_.begin(), tracks_.end(),
                [best_id](const Track& t) {
                    return t.target.track_id == best_id;
                });
            if (it != tracks_.end()) {
                best_track_ = it->target;
                return;
            }
        }

        if (state_ != TrackerState::LOST) {
            state_ = TrackerState::LOST;
        }
        best_track_ = Target{};
        best_track_.state = TrackerState::LOST;
        best_track_.track_id = -1;
    }

    Target best_track_target() const {
        return best_track_;
    }

    TrackerState state_ = TrackerState::LOST;
    Target best_track_;
    std::vector<Track> tracks_;
    int next_id_ = 0;
    int min_detect_count_ = 3;
    int max_lost_count_ = 10;
    int max_tracks_ = 6;
};

Tracker::Tracker(const std::string& config_path)
    : impl_(std::make_unique<Impl>(config_path)) {}

Tracker::~Tracker() = default;

Target Tracker::update(const std::vector<ArmorPose>& armors, int frame_id) {
    return impl_->update(armors, frame_id);
}

TrackerState Tracker::state() const {
    return impl_->state();
}

const Target& Tracker::tracked_target() const {
    return impl_->tracked_target();
}
