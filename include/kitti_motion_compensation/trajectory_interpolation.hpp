#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc::trajectory_interpolation {

Affine3d InterpolateTrajectory(Oxts const &odometry_1, Oxts const &odometry_2, Time const time);

class TrajectoryInterpolator {
 public:
  TrajectoryInterpolator(Oxts const &odometry_1, Oxts const &odometry_2);

  TrajectoryInterpolator(Time const time_1, Affine3d const &pose_1, Time const time_2, Affine3d const &pose_2);

  Affine3d GetPoseAtTime(Time const time) const;

  Affine3d RelativePoseBetweenTimes(Time const anchor_time, Time const query_time) const;

 private:
  bool TimeIsInRange(Time const time) const;

  double FractionOfTrajectory(Time const time) const;

  Time time_1_;
  Affine3d pose_1_;

  Time time_2_;
  Affine3d pose_2_;
};

}  // namespace kmc::trajectory_interpolation