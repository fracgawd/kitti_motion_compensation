#include "kitti_motion_compensation/motion_compensation.hpp"

#include "kitti_motion_compensation/lie_algebra.hpp"

namespace kmc {

Vector4d MotionCompensatePoint(Vector4d const& point, Twist const& delta_pose, double const x) {
  // This function considers that there is some trajectory, in between two poses, that we want to interpolate on. The
  // total distance between the two poses is `delta_pose` and `x` is the fraction of total distance we actually want to
  // transform the point with.
  //
  // TODO(jack): what are the valid values of x?

  Affine3d const tf_delta{InterpolatedPose(delta_pose, x)};

  return (tf_delta * point);
}

Vector4d MotionCompensateFramePoint(Vector4d const& point, Time const point_stamp, Twist const& frame_delta_pose,
                                    Time const frame_start, Time const frame_end, Time const requested_time) {
  // TODO(jack): what are the valid values of x?
  // TODO(jack): use single time fraction interpolation function?

  double const x{(requested_time - point_stamp) / (frame_end - frame_start)};

  return MotionCompensatePoint(point, frame_delta_pose, x);
}

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time) {
  LidarScan const& scan{frame.scan};
  Time const frame_start{scan.stamp_start};
  Time const frame_end{scan.stamp_end};

  // calculate the total motion delta experienced during the frame capture. The compensation applied to all points in
  // the frame will be some fraction of this value.
  Twist const frame_delta_pose{DeltaPose(frame.T_start, frame.T_end)};

  Index const num_points{scan.cloud.rows()};
  Pointcloud motion_compensated_cloud = MatrixX4d(num_points, 4);
  for (Index i{0}; i < num_points; ++i) {
    Vector4d const point_i{scan.cloud.row(i)};
    Time const stamp_i{scan.timestamps(i)};

    motion_compensated_cloud.row(i) =
        MotionCompensateFramePoint(point_i, stamp_i, frame_delta_pose, frame_start, frame_end, requested_time);
  }

  return motion_compensated_cloud;
}

}  // namespace kmc