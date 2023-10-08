#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

double FractionOfScanCompleted(Eigen::Vector4d const point);

Time GetPseudoTimeStamp(Eigen::Vector4d const point, Time const scan_start, Time const scan_end);

VectorXd GetPseudoTimeStamps(Pointcloud const &cloud, Time const start_time, Time const end_time);

}  // namespace kmc
