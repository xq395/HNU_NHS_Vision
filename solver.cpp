#include "solve/solver.hpp"
#include "solve/extrinsics.hpp"
#include "common/config.hpp"
#include "common/logger.hpp"
#include "common/types.hpp"

#include <opencv2/calib3d.hpp>

class Solver::Impl {
public:
    Impl(const std::string& config_path)
        : extrinsics_(config_path)
    {
        Config cfg(config_path);

        auto node = cfg.get_node("solver");
        if (node) {
            small_armor_width_ = node["small_armor_width"].as<double>(0.230);
            small_armor_height_ = node["small_armor_height"].as<double>(0.054);
            large_armor_width_ = node["large_armor_width"].as<double>(0.225);
            large_armor_height_ = node["large_armor_height"].as<double>(0.054);
            bullet_speed_ = node["bullet_speed"].as<double>(15.0);
            max_reprojection_error_ = node["max_reprojection_error"].as<double>(10.0);
        }

        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = cfg.get<double>("camera.fx", 1.0);
        camera_matrix_.at<double>(1, 1) = cfg.get<double>("camera.fy", 1.0);
        camera_matrix_.at<double>(0, 2) = cfg.get<double>("camera.cx", 0.0);
        camera_matrix_.at<double>(1, 2) = cfg.get<double>("camera.cy", 0.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }

    SolveResult solve(const DetectionResult& det,
                      const cv::Mat& camera_matrix,
                      const cv::Mat& dist_coeffs,
                      const Eigen::Quaterniond& imu_orientation)
    {
        SolveResult result;
        result.frame_id = det.frame_id;
        result.timestamp = det.timestamp;

        for (const auto& armor : det.armors) {
            ArmorPose pose;

            std::vector<cv::Point3f> object_points = (armor.type == ArmorType::LARGE)
                ? get_large_armor_points()
                : get_small_armor_points();

            std::vector<cv::Point2f> image_points = {
                cv::Point2f(armor.points[0].x, armor.points[0].y),
                cv::Point2f(armor.points[1].x, armor.points[1].y),
                cv::Point2f(armor.points[2].x, armor.points[2].y),
                cv::Point2f(armor.points[3].x, armor.points[3].y)
            };

            cv::Mat rvec, tvec;
            if (!cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                              rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
                continue;
            }

            double reproj_error = compute_reprojection_error(
                object_points, image_points, rvec, tvec, camera_matrix, dist_coeffs);

            if (reproj_error > max_reprojection_error_) {
                continue;
            }

            cv::Mat rot_mat;
            cv::Rodrigues(rvec, rot_mat);

            Eigen::Vector3d position_cam(tvec.at<double>(0),
                                         tvec.at<double>(1),
                                         tvec.at<double>(2));

            Eigen::Matrix3d R_cam2gimbal = extrinsics_.rotation();
            Eigen::Vector3d t_cam2gimbal = extrinsics_.translation();
            Eigen::Vector3d position_gimbal = R_cam2gimbal * position_cam + t_cam2gimbal;

            Eigen::Vector3d position_world = imu_orientation * position_gimbal;

            double roll, pitch, yaw;
            rot_to_euler(rot_mat, roll, pitch, yaw);
            yaw = resolve_yaw_ambiguity(yaw, rot_mat);

            pose.position = position_world;
            pose.rotation = Eigen::Vector3d(roll, pitch, yaw);
            pose.type = armor.type;
            pose.name = armor_name_from_points(armor.points, armor.center, armor.type);
            pose.yaw = static_cast<float>(yaw);
            pose.reprojection_error = static_cast<float>(reproj_error);

            result.armors.push_back(pose);
        }

        return result;
    }

private:
    static std::vector<cv::Point3f> get_small_armor_points() {
        float hw = 0.230f / 2.0f;
        float hh = 0.054f / 2.0f;
        return {
            cv::Point3f(-hw, -hh, 0),
            cv::Point3f( hw, -hh, 0),
            cv::Point3f( hw,  hh, 0),
            cv::Point3f(-hw,  hh, 0)
        };
    }

    static std::vector<cv::Point3f> get_large_armor_points() {
        float hw = 0.225f / 2.0f;
        float hh = 0.054f / 2.0f;
        return {
            cv::Point3f(-hw, -hh, 0),
            cv::Point3f( hw, -hh, 0),
            cv::Point3f( hw,  hh, 0),
            cv::Point3f(-hw,  hh, 0)
        };
    }

    static double compute_reprojection_error(
        const std::vector<cv::Point3f>& obj_pts,
        const std::vector<cv::Point2f>& img_pts,
        const cv::Mat& rvec, const cv::Mat& tvec,
        const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs, projected);

        double error = 0.0;
        for (size_t i = 0; i < img_pts.size(); ++i) {
            error += cv::norm(img_pts[i] - projected[i]);
        }
        return error / img_pts.size();
    }

    static void rot_to_euler(const cv::Mat& R, double& roll, double& pitch, double& yaw) {
        pitch = std::atan2(-R.at<double>(2, 0),
                           std::sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) +
                                     R.at<double>(2, 2) * R.at<double>(2, 2)));
        yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    }

    static float resolve_yaw_ambiguity(float yaw, const cv::Mat& rot_mat) {
        Eigen::Vector3d forward = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d normal(
            rot_mat.at<double>(0, 2),
            rot_mat.at<double>(1, 2),
            rot_mat.at<double>(2, 2));

        if (normal.dot(forward) < 0) {
            yaw += static_cast<float>(PI);
        }

        while (yaw > PI) yaw -= 2.0f * static_cast<float>(PI);
        while (yaw < -PI) yaw += 2.0f * static_cast<float>(PI);

        return yaw;
    }

    static ArmorName armor_name_from_points(
        const std::array<cv::Point2f, 4>& points,
        const cv::Point2f& center,
        ArmorType type)
    {
        return ArmorName::UNKNOWN;
    }

    Extrinsics extrinsics_;
    double small_armor_width_ = 0.230;
    double small_armor_height_ = 0.054;
    double large_armor_width_ = 0.225;
    double large_armor_height_ = 0.054;
    double bullet_speed_ = 15.0;
    double max_reprojection_error_ = 10.0;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

Solver::Solver(const std::string& config_path)
    : impl_(std::make_unique<Impl>(config_path)) {}

Solver::~Solver() = default;

SolveResult Solver::solve(const DetectionResult& det,
                          const cv::Mat& camera_matrix,
                          const cv::Mat& dist_coeffs,
                          const Eigen::Quaterniond& imu_orientation)
{
    return impl_->solve(det, camera_matrix, dist_coeffs, imu_orientation);
}
