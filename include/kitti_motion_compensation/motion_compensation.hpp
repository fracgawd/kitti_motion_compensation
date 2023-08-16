#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

bool DataIsSynchronized(Time const stamp_1, Time const stamp_2, Time const stamp_3);

double FractionOfScanCompleted(Eigen::Vector4d const point);

Time GetPseudoTimeStamp(Eigen::Vector4d const point, Time const scan_start, Time const scan_end);

Eigen::Vector4d MotionCompensatePoint(Eigen::Vector4d const point, Time const point_stamp, Oxts const odometry,
                                      Time const requested_time);

Pointcloud MotionCompensate(Frame frame, Time const requested_time);

Pointcloud MotionCompensate(LidarScan const &lidar_scan, Oxts const odometry, Time const requested_time);

}  // namespace kmc
