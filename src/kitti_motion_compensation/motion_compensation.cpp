#include "kitti_motion_compensation/motion_compensation.hpp"

#include "kitti_motion_compensation/trajectory_interpolation.hpp"

namespace kmc {

using TrajectoryInterpolator = trajectory_interpolation::TrajectoryInterpolator;

Vector4d MotionCompensatePoint(TrajectoryInterpolator const& trajectory_interpolator, Time const point_stamp,
                               Vector4d const& point, Time const requested_time) {
  Affine3d const correction{trajectory_interpolator.RelativePoseBetweenTimes(requested_time, point_stamp)};

  return correction * point;
}

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time) {
  TrajectoryInterpolator const trajectory_interpolator(frame.scan.stamp_start, frame.T_start, frame.scan.stamp_end,
                                                       frame.T_end);

  Index const num_points{frame.scan.cloud.rows()};
  Pointcloud motion_compensated_cloud{MatrixX4d(num_points, 4)};
  for (Index i{0}; i < num_points; ++i) {
    motion_compensated_cloud.row(i) = MotionCompensatePoint(trajectory_interpolator, frame.scan.timestamps(i),
                                                            frame.scan.cloud.row(i), requested_time);
  }

  return motion_compensated_cloud;
}

}  // namespace kmc