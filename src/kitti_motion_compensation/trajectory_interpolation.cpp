#include "kitti_motion_compensation/trajectory_interpolation.hpp"

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/lie_algebra.hpp"

namespace kmc {

// TODO(jack): test all the lie algebra handlers

Twist DeltaPose(Affine3d const &pose_0, Affine3d const &pose_1) { return lie::Log(pose_0.inverse() * pose_1); }

Affine3d InterpolatedPose(Twist const &delta_pose, double const x) {
  Twist const delta_pose_x{x * delta_pose};  // se3

  return lie::Exp(delta_pose_x);  // SE3
}

Affine3d InterpolatePose(Affine3d const &pose_0, Affine3d const &pose_1, double const x) {
  // Given two poses, pose_0 and pose_1, get a pose somewhere in between. The fraction of the distance between the two
  // poses is specified by "x" (0<x<1)
  //
  // If you want to think about is on a very simplified level, but that still capture the essence, this function does
  // the following (poses don't really work like this, but it helps understanding -_-):
  //
  //      pose_interpolated = pose_0 + (pose_1 - pose_0) * x

  if ((x < 0) or (1 < x)) {
    std::cerr << "You gave an invalid interpolation fraction: " << x
              << " -_- you are only allowed to interpolate within two poses, "
                 "not further than that."
              << std::endl;
    throw std::invalid_argument("314524635");
  }

  Twist const delta_pose{DeltaPose(pose_0, pose_1)};
  Affine3d const tf_delta{InterpolatedPose(delta_pose, x)};

  return pose_0 * tf_delta;
}

Affine3d InterpolateFramePose(Oxts const &odometry_0, Oxts const &odometry_1, Time requested_time) {
  // This if condition will really only handle the case when an edge frame (N=0 or N=n) has the same odometry loaded
  // twice. Then we can just return the same pose, and protect the function from a divide by zero below because the
  // timestamps of the odometry will be the same.
  if (odometry_0.stamp == odometry_1.stamp) {
    return OxtsToPose(odometry_0);  // could also return the pose from `odometry_1`, they are the same
  }

  double const x{(requested_time - odometry_0.stamp) / (odometry_1.stamp - odometry_0.stamp)};

  return InterpolatePose(OxtsToPose(odometry_0), OxtsToPose(odometry_1), x);
}

}  // namespace kmc