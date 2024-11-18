#include "lbr_fri_ros2/control.hpp"

namespace lbr_fri_ros2 {
InvJacCtrlImpl::InvJacCtrlImpl(const std::string &robot_description,
                               const InvJacCtrlParameters &parameters)
    : parameters_(parameters) {
  kinematics_ptr_ = std::make_unique<Kinematics>(robot_description, parameters_.chain_root,
                                                 parameters_.chain_tip);
}

void InvJacCtrlImpl::compute(const geometry_msgs::msg::Twist::SharedPtr &twist_target,
                             const_jnt_array_t_ref q, jnt_array_t_ref dq) {
  // twist to Eigen
  twist_target_[0] = twist_target->linear.x;
  twist_target_[1] = twist_target->linear.y;
  twist_target_[2] = twist_target->linear.z;
  twist_target_[3] = twist_target->angular.x;
  twist_target_[4] = twist_target->angular.y;
  twist_target_[5] = twist_target->angular.z;

  // compute
  compute_impl_(q, dq);
}

void InvJacCtrlImpl::compute(const_cart_array_t_ref twist_target, const_jnt_array_t_ref q,
                             jnt_array_t_ref dq) {
  // twist to Eigen
  twist_target_[0] = twist_target[0];
  twist_target_[1] = twist_target[1];
  twist_target_[2] = twist_target[2];
  twist_target_[3] = twist_target[3];
  twist_target_[4] = twist_target[4];
  twist_target_[5] = twist_target[5];

  // compute
  compute_impl_(q, dq);
}

void InvJacCtrlImpl::compute(const Eigen::Matrix<double, CARTESIAN_DOF, 1> &twist_target,
                             const_jnt_array_t_ref q, jnt_array_t_ref dq) {
  twist_target_ = twist_target;

  // compute
  compute_impl_(q, dq);
}

void InvJacCtrlImpl::compute_impl_(const_jnt_array_t_ref q, jnt_array_t_ref dq) {
  // clip velocity
  twist_target_.head(3).unaryExpr([&](double v) {
    return std::clamp(v, -parameters_.max_linear_velocity, parameters_.max_linear_velocity);
  });
  twist_target_.tail(3).unaryExpr([&](double v) {
    return std::clamp(v, -parameters_.max_angular_velocity, parameters_.max_angular_velocity);
  });

  // if desired, transform to tip frame
  if (parameters_.twist_in_tip_frame) {
    auto chain_tip_frame = kinematics_ptr_->compute_fk(q);
    twist_target_.topRows(3) =
        Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * twist_target_.topRows(3);
    twist_target_.bottomRows(3) =
        Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * twist_target_.bottomRows(3);
  }

  // compute jacobian
  auto jacobian = kinematics_ptr_->compute_jacobian(q);
  jacobian_inv_ = pinv(jacobian.data, parameters_.damping);

  // compute target joint veloctiy and map it to dq
  Eigen::Map<Eigen::Matrix<double, N_JNTS, 1>>(dq.data()) = jacobian_inv_ * twist_target_;
}
} // namespace lbr_fri_ros2
