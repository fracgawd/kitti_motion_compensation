#include "kitti_motion_compensation/trajectory_interpolation.hpp"

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/lie_algebra.hpp"

#undef NDEBUG  // keep asserts in release mode
#include <cassert>

namespace kmc::trajectory_interpolation {

Affine3d InterpolateTrajectory(Oxts const &odometry_1, Oxts const &odometry_2, Time const time) {
  TrajectoryInterpolator const trajectory_interpolator{
      trajectory_interpolation::TrajectoryInterpolator(odometry_1, odometry_2)};

  return trajectory_interpolator.GetPoseAtTime(time);
}

TrajectoryInterpolator::TrajectoryInterpolator(Oxts const &odometry_0, Oxts const &odometry_1)
    : time_1_{odometry_0.stamp},
      pose_1_{OxtsToPose(odometry_0)},
      time_2_{odometry_1.stamp},
      pose_2_{OxtsToPose(odometry_1)} {}

TrajectoryInterpolator::TrajectoryInterpolator(Time const time_1, Affine3d const &pose_1, Time const time_2,
                                               Affine3d const &pose_2)
    : time_1_{time_1}, pose_1_{pose_1}, time_2_{time_2}, pose_2_{pose_2} {}

Affine3d TrajectoryInterpolator::GetPoseAtTime(Time const time) const {
  assert(TimeIsInRange(time) and "You gave a time outside of the two poses you wanted to interpolate between :(");

  // TODO(jack): make interpretable :)
  Twist const f{lie::Log(pose_1_.inverse() * pose_2_)};
  double const x{FractionOfTrajectory(time)};
  Twist const f_x{x * f};
  Affine3d const T_x{lie::Exp(f_x)};

  return pose_1_ * T_x;
}

Affine3d TrajectoryInterpolator::RelativePoseBetweenTimes(Time const anchor_time, Time const query_time) const {
  return GetPoseAtTime(anchor_time).inverse() * GetPoseAtTime(query_time);
}

bool TrajectoryInterpolator::TimeIsInRange(Time const time) const { return (time >= time_1_) and (time <= time_2_); }

double TrajectoryInterpolator::FractionOfTrajectory(Time const time) const {
  return (time - time_1_) / (time_2_ - time_1_);
}

}  // namespace kmc::trajectory_interpolation