#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Twist DeltaPose(Affine3d const &pose_0, Affine3d const &pose_1);

Affine3d InterpolatedPose(Twist const &delta_pose, double const x);

Affine3d InterpolatePose(Affine3d const &pose_0, Affine3d const &pose_1, double const x);

Affine3d InterpolateFramePose(Oxts const &odometry_0, Oxts const &odometry_1, Time requested_time);

}  // namespace kmc

namespace kmc::trajectory_interpolation {
class TrajectoryInterpolator {
 public:
  TrajectoryInterpolator(Oxts const &odometry_0, Oxts const &odometry_1);

  Affine3d GetPoseAtTime(Time const time) const;

 private:
  bool TimeIsInRange(Time const time) const;

  double FractionOfTrajectory(Time const time) const;

  Time time_0_;
  Affine3d pose_0_;

  Time time_1_;
  Affine3d pose_1_;
};
}  // namespace kmc::trajectory_interpolation