#include "kitti_motion_compensation/motion_compensation.hpp"

namespace kmc {

bool DataIsSynchronized(Time const stamp_1, Time const stamp_2,
                        Time const stamp_3) {
  // The lidar scans at 10Hz and the data is all synchonrized, so the timestamp
  // gap should never be larger than 0.1s. It is probably usually much smaller
  // actually :)
  return (std::abs(stamp_1 - stamp_2) < 0.10) and
         (std::abs(stamp_2 - stamp_3) < 0.10);
}

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
  //    default
  //        value) and its time in the scan be recovered from its known
  //        position
  //        in the data structure.
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

Time GetPseudoTimeStamp(Eigen::Vector4d const point, Time const scan_start,
                        Time const scan_end) {
  double const position_in_scan{FractionOfScanCompleted(point)};
  Time const scan_duration{scan_end - scan_start};

  return scan_start + (scan_duration * position_in_scan);
}

Eigen::Vector4d MotionCompensatePoint(Eigen::Vector4d const point,
                                      Time const point_stamp,
                                      Oxts const odometry,
                                      Time const requested_time) {

  Time const delta_time{point_stamp - requested_time};

  Eigen::Vector3d const delta_pose{
      delta_time * Eigen::Vector3d{odometry.vf, odometry.vl, odometry.vu}};

  Eigen::Vector4d motion_compensated_point{point};
  motion_compensated_point.topRows(3) =
      delta_pose + motion_compensated_point.topRows(3);

  return motion_compensated_point;
}

Pointcloud MotionCompensate(Frame frame, Time const requested_time) {
  return MotionCompensate(frame.scan_, frame.odometry_, requested_time);
}

Pointcloud MotionCompensate(LidarScan const &lidar_scan, Oxts const odometry,
                            Time const requested_time) {
  if (not DataIsSynchronized(lidar_scan.stamp_middle, odometry.stamp,
                             requested_time)) {
    std::cout
        << "The requested synchronization was not within the time tolerance. "
           "Are you sure your requested time is within the LidarScan? Are "
           "you sure your odometry is for the LidarScan you provided?"
        << '\n';
    exit(0);
  }

  Pointcloud const &cloud{lidar_scan.cloud};
  Time const scan_start{lidar_scan.stamp_start};
  Time const scan_end{lidar_scan.stamp_end};

  Pointcloud motion_compensated_cloud = Eigen::MatrixX4d(cloud.rows(), 4);
  for (Eigen::Index i{0}; i < cloud.rows(); ++i) {
    Eigen::Vector4d const point_i{cloud.row(i)};
    Time const point_i_stamp{GetPseudoTimeStamp(point_i, scan_start, scan_end)};

    Eigen::Vector4d const point_i_motion_compensated{MotionCompensatePoint(
        point_i, point_i_stamp, odometry, requested_time)};

    motion_compensated_cloud.row(i) = point_i_motion_compensated;
  }

  return motion_compensated_cloud;
}

} // namespace kmc