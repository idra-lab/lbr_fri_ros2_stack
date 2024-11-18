#ifndef LBR_FRI_ROS2__CONTROL_HPP_
#define LBR_FRI_ROS2__CONTROL_HPP_

#include <algorithm>
#include <memory>
#include <string>

#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/twist.hpp"

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
public:
  InvJacCtrlImpl(const std::string &robot_description, const InvJacCtrlParameters &parameters);

  void compute(const geometry_msgs::msg::Twist::SharedPtr &twist_target, const_jnt_array_t_ref q,
               jnt_array_t_ref dq);
  void compute(const_cart_array_t_ref twist_target, const_jnt_array_t_ref q, jnt_array_t_ref dq);
  void compute(const Eigen::Matrix<double, CARTESIAN_DOF, 1> &twist_target, const_jnt_array_t_ref q,
               jnt_array_t_ref dq);

  inline const std::unique_ptr<Kinematics> &get_kinematics_ptr() const { return kinematics_ptr_; };

protected:
  void compute_impl_(const_jnt_array_t_ref q, jnt_array_t_ref dq);

  InvJacCtrlParameters parameters_;

  jnt_array_t q_;
  std::unique_ptr<Kinematics> kinematics_ptr_;
  Eigen::Matrix<double, N_JNTS, CARTESIAN_DOF> jacobian_inv_;
  Eigen::Matrix<double, CARTESIAN_DOF, 1> twist_target_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__CONTROL_HPP_
