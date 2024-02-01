#include "kitti_motion_compensation/motion_compensation.hpp"

#include "kitti_motion_compensation/trajectory_interpolation.hpp"

namespace kmc {

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time) {
  LidarScan const& scan{frame.scan};
  Time const frame_start{scan.stamp_start};
  Time const frame_end{scan.stamp_end};

  trajectory_interpolation::TrajectoryInterpolator const trajectory_interpolator(frame_start, frame.T_start, frame_end,
                                                                                 frame.T_end);

  Index const num_points{scan.cloud.rows()};
  Pointcloud motion_compensated_cloud = MatrixX4d(num_points, 4);
  for (Index i{0}; i < num_points; ++i) {
    Vector4d const point_i{scan.cloud.row(i)};
    Time const stamp_i{scan.timestamps(i)};

    auto const pose_at_stamp_i{trajectory_interpolator.GetPoseAtTime(stamp_i)};

    motion_compensated_cloud.row(i) =
        trajectory_interpolator.RelativePoseBetweenTimes(requested_time, stamp_i) * point_i;
  }

  return motion_compensated_cloud;
}

}  // namespace kmc