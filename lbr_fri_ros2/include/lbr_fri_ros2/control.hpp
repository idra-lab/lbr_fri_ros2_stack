#ifndef LBR_FRI_ROS2__CONTROL_HPP_
#define LBR_FRI_ROS2__CONTROL_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "lbr_fri_ros2/kinematics.hpp"
#include "lbr_fri_ros2/pinv.hpp"
#include "lbr_fri_ros2/types.hpp"

namespace lbr_fri_ros2 {
struct InvJacCtrlParameters {
  std::string chain_root;
  std::string chain_tip;
  bool twist_in_tip_frame;
  double damping;
  double max_linear_velocity;
  double max_angular_velocity;
};

class InvJacCtrlImpl {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::InvJacCtrlImpl";

public:
  InvJacCtrlImpl(const std::string &robot_description, const InvJacCtrlParameters &parameters);

  void compute(const geometry_msgs::msg::Twist::SharedPtr &twist_target, const_jnt_array_t_ref q,
               jnt_array_t_ref dq);
  void compute(const_cart_array_t_ref twist_target, const_jnt_array_t_ref q, jnt_array_t_ref dq);
  void compute(const Eigen::Matrix<double, CARTESIAN_DOF, 1> &twist_target, const_jnt_array_t_ref q,
               jnt_array_t_ref dq);

  inline const std::unique_ptr<Kinematics> &get_kinematics_ptr() const { return kinematics_ptr_; };

  void log_info() const;

protected:
  void compute_impl_(const_jnt_array_t_ref q, jnt_array_t_ref dq);

  InvJacCtrlParameters parameters_;

  jnt_array_t q_;
  std::unique_ptr<Kinematics> kinematics_ptr_;
  Eigen::Matrix<double, N_JNTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> twist_target_;
};

struct AdmittanceParameters {
  AdmittanceParameters() = delete;
  AdmittanceParameters(const double &m = 1.0, const double &b = 0.1, const double &k = 0.0)
      : m(m), b(b), k(k) {
    if (m <= 0.0) {
      throw std::runtime_error("Mass must be positive and greater zero.");
    }
    if (b < 0.0) {
      throw std::runtime_error("Damping must be positive.");
    }
    if (k < 0.0) {
      throw std::runtime_error("Stiffness must be positive.");
    }
  }

  double m = 1.0;
  double b = 0.1;
  double k = 0.0;
};

class AdmittanceImpl {
protected:
  static constexpr char LOGGER_NAME[] = "lbr_fri_ros2::AdmittanceImpl";

public:
  AdmittanceImpl(const AdmittanceParameters &parameters) : parameters_(parameters) {}

  void compute(const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &f_ext,
               const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &x,
               const Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &dx,
               Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> &ddx);

  void log_info() const;

protected:
  AdmittanceParameters parameters_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__CONTROL_HPP_
