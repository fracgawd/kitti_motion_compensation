
#include "kitti_motion_compensation/timestamp_mocking.hpp"

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

VectorXd GetPseudoTimeStamps(Pointcloud const &cloud, Time const start_time, Time const end_time) {
  VectorXd timestamps = VectorXd(cloud.rows());
  for (Eigen::Index i{0}; i < cloud.rows(); ++i) {
    timestamps(i) = GetPseudoTimeStamp(cloud.row(i), start_time, end_time);
  }

  return timestamps;
}

}  // namespace kmc