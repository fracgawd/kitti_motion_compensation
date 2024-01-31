#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Twist DeltaPose(Affine3d const &pose_0, Affine3d const &pose_1);

Affine3d InterpolatedPose(Twist const &delta_pose, double const x);

Affine3d InterpolatePose(Affine3d const &pose_0, Affine3d const &pose_1, double const x);

Affine3d InterpolateFramePose(kmc::Oxts const &odometry_0, kmc::Oxts const &odometry_1, kmc::Time requested_time);

}  // namespace kmc
