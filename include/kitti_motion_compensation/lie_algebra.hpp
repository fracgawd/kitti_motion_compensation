#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

// the lie algebra documentation was adopted based on the
// https://github.com/utiasSTARS/liegroups/blob/master/liegroups/

namespace kmc::lie {

Eigen::Matrix3d Hat(Eigen::Vector3d const& a);

Eigen::Vector3d Vee(Eigen::Matrix3d const& a);

Eigen::Matrix3d Exp(Eigen::Vector3d const& phi);

Eigen::Vector3d Log(Eigen::Matrix3d const& R);

Eigen::Matrix3d LeftJacobian(Eigen::Vector3d const& phi);

Eigen::Matrix3d InverseLeftJacobian(Eigen::Vector3d const& phi);

Eigen::Affine3d Exp(Twist const& xi);

Twist Log(Eigen::Affine3d const& T);

}  // namespace kmc::lie
