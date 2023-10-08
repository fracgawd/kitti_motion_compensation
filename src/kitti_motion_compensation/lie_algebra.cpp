#include "kitti_motion_compensation/lie_algebra.hpp"

#include <cmath>

namespace kmc::lie {

Eigen::Matrix3d Hat(Eigen::Vector3d const& a) {
  Eigen::Matrix3d a_hat{Eigen::Matrix3d::Zero()};

  a_hat(0, 1) = -a(2);
  a_hat(0, 2) = a(1);
  a_hat(1, 0) = a(2);
  a_hat(1, 2) = -a(0);
  a_hat(2, 0) = -a(1);
  a_hat(2, 1) = a(0);

  return a_hat;
}

Eigen::Vector3d Vee(Eigen::Matrix3d const& a) { return Eigen::Vector3d{a(2, 1), a(0, 2), a(1, 0)}; }

Eigen::Matrix3d Exp(Eigen::Vector3d const& phi) {
  double const angle{phi.norm()};

  // use first order taylor expansion when phi is small
  if (angle < 1e-6) {
    return (Eigen::Matrix3d::Identity() + Hat(phi));
  }

  Eigen::Vector3d const axis{phi / angle};
  double const c{std::cos(angle)};
  double const s{std::sin(angle)};

  return ((c * Eigen::Matrix3d::Identity()) + ((1.0 - c) * axis * axis.transpose()) + (s * Hat(axis)));
}

Eigen::Vector3d Log(Eigen::Matrix3d const& R) {
  double c{(0.5 * R.trace()) - 0.5};
  c = std::clamp(c, -1.0, 1.0);

  double const angle{std::acos(c)};

  // use first order taylor expansion when angle is small
  if (angle < 1e-6) {
    return Vee(R - Eigen::Matrix3d::Identity());
  }

  return Vee((0.5 * angle / std::sin(angle)) * (R - R.transpose()));
}

Eigen::Matrix3d LeftJacobian(Eigen::Vector3d const& phi) {
  double const angle{phi.norm()};

  // use first order taylor expansion when phi is small
  if (angle < 1e-6) {
    return (Eigen::Matrix3d::Identity() + (0.5 * Hat(phi)));
  }

  Eigen::Vector3d const axis{phi / angle};
  double const c{std::cos(angle)};
  double const s{std::sin(angle)};

  return (((s / angle) * Eigen::Matrix3d::Identity()) + ((1.0 - (s / angle)) * (axis * axis.transpose())) +
          (((1 - c) / angle) * Hat(axis)));
}

Eigen::Matrix3d InverseLeftJacobian(Eigen::Vector3d const& phi) {
  double const angle{phi.norm()};

  // use first order taylor expansion when phi is small
  if (angle < 1e-6) {
    return (Eigen::Matrix3d::Identity() - (0.5 * Hat(phi)));
  }

  Eigen::Vector3d const axis{phi / angle};
  double const half_angle{0.5 * angle};
  double const half_angle_cotanget{1.0 / std::tan(half_angle)};

  return ((half_angle * half_angle_cotanget * Eigen::Matrix3d::Identity()) +
          ((1 - (half_angle * half_angle_cotanget)) * (axis * axis.transpose())) - (half_angle * Hat(axis)));
}

Eigen::Affine3d Exp(Twist const& xi) {
  Eigen::Vector3d const rho{xi.topRows(3)};
  Eigen::Vector3d const phi{xi.bottomRows(3)};

  Eigen::Affine3d T{Eigen::Affine3d::Identity()};
  T *= Exp(phi);                               // put rotation in T
  T.translation() << LeftJacobian(phi) * rho;  // put translation in T

  return T;
}

Twist Log(Eigen::Affine3d const& T) {
  Eigen::Vector3d const phi{Log(T.rotation().matrix())};
  Eigen::Vector3d const rho{InverseLeftJacobian(phi) * T.translation()};

  Twist v;
  v.topRows(3) = rho;
  v.bottomRows(3) = phi;

  return v;
}

}  // namespace kmc::lie

namespace kmc {
// TODO(jack): test all the lie algebra handlers

Twist DeltaPose(Affine3d const& pose_0, Affine3d const& pose_1) { return lie::Log(pose_0.inverse() * pose_1); }

Affine3d InterpolatedPose(Twist const& delta_pose, double const x) {
  Twist const delta_pose_x{x * delta_pose};  // se3

  return lie::Exp(delta_pose_x);  // SE3
}

}  // namespace kmc