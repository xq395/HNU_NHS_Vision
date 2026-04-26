#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace HNU_NHS_Vision::auto_aim{
 struct Armor {
    public:
        ArmorNumber number;
        std::string type;
        Eigen::Vector3d pos;
        Eigen::Quaterniond ori;
        Eigen::Vector3d target_pos;
        Eigen::Quaterniond target_ori;
        float distance_to_image_center;
        float yaw;
        std::chrono::steady_clock::time_point timestamp;
        bool is_ok = false;
        bool is_none_purple = false;
        int id = -1;
        std::vector<cv::Point2f> toPtsDebug(
            const cv::Mat& camera_intrinsic,
            const cv::Mat& camera_distortion
        ) const noexcept;
    };
    struct Armors {
    public:
        std::vector<Armor> armors;
        std::chrono::steady_clock::time_point timestamp;
        int id;
        Eigen::Vector3d v;
    };
    
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};
}
