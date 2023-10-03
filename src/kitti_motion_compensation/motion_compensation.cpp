#include "kitti_motion_compensation/motion_compensation.hpp"

#include "kitti_motion_compensation/lie_algebra.hpp"

namespace kmc {

double FractionOfScanCompleted(Eigen::Vector4d const point) {
  // This function is where we deal with the fundmanetal problem that makes
  // the kitti lidar hard to motion compensate for most people.
  //
  // The most important thing about being able to motion compensate a
  // pointcloud is understanding when in the scan each point was captured. There
  // are two easy ways to get this information:
  //
  //    1) Each point has a time stamp.
  //    2) The lidar scan is "dense" (i.e. non-returns are filled with a
  //    default value) and its time in the scan be recovered from its known
  //    position in the data structure.
  //
  // Unfortunately for us, the kitti lidar data has neither of the above
  // properties. So then how do we recover "when in the scan" each point was
  // captured?
  //
  // We know that the lidar is rotating around its vertical axis, and
  // it starts the rotation scanning at the back of vehicle, rotates counter
  // clockwise (looking from birds eye view down onto the car) and finally
  // ends the scan at the same location at the back of the vehicle where it
  // started. It does this every 0.1s, collecting data in a sweeping broom
  // pattern, where the data at every vertical colum is essentially captured at
  // one moment in time.
  //
  // Understanding this fully then, we can essentially calculate a pseudo
  // timestamp using the coordinate of each point itself. We take the (x,y)
  // value and calclute the arctan2 of that point. This gives a value in
  // between -pi and pi that represents where in the scan the point is, and with
  // some simpel math and dividing by the full circle (2pi), we can get the
  // "fraction of scan completed" where that point is. If we know how long each
  // scan took (~0.1s) then we can calculate a pseudo time stamp for that point.
  //
  // Of course this assumes the lidar is parallel to the ground (mostly true)
  // and that it is really aligned directly with the forward facing direction.
  // Both assumptions in practice that we find to be reasonable.
  //
  // This function is my best attempt of getting the temporal position of each
  // point within the scan with the means at my disposal :)

  return (M_PI + std::atan2(point(1), point(0))) / (2.0 * M_PI);
}

Time GetPseudoTimeStamp(Eigen::Vector4d const point, Time const scan_start, Time const scan_end) {
  double const position_in_scan{FractionOfScanCompleted(point)};
  Time const scan_duration{scan_end - scan_start};

  return scan_start + (position_in_scan * scan_duration);
}

Vector4d MotionCompensatePoint(Vector4d const& point, Twist const& delta_pose, double const x) {
  // This function considers that there is some trajectory, in between two poses, that we want to interpolate on. The
  // total distance between the two poses is `delta_pose` and `x` is the fraction of total distance we actually want to
  // transform the point with.
  //
  // TODO(jack): what are the valid values of x?

  Twist const delta_x{x * delta_pose};         // se3
  Affine3d const tf_delta{lie::Exp(delta_x)};  // SE3

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
  Twist const frame_delta_pose{kmc::lie::Log(frame.T_start.inverse() * frame.T_end)};

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