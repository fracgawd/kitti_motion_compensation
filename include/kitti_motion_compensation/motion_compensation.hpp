#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

bool DataIsSynchronized(Time const stamp_1, Time const stamp_2, Time const stamp_3);

double FractionOfScanCompleted(Eigen::Vector4d const point);

Time GetPseudoTimeStamp(Eigen::Vector4d const point, Time const scan_start, Time const scan_end);

Vector4d MotionCompensatePoint(Vector4d const& point, Twist const& delta_pose, double const x);

Vector4d MotionCompensateFramePoint(Vector4d const& point, Time const point_stamp, Twist const& frame_delta_pose,
                                    Time const frame_start, Time const frame_end, Time const requested_time);

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time);

}  // namespace kmc
