#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

double ScaledTimeDisplacment(Time const query_time, Time const anchor_time, Time const min_time, Time const max_time);

Vector4d MotionCompensatePoint(Vector4d const& point, Twist const& delta_pose, double const x);

Vector4d MotionCompensateFramePoint(Vector4d const& point, Time const point_stamp, Twist const& frame_delta_pose,
                                    Time const frame_start, Time const frame_end, Time const requested_time);

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time);

}  // namespace kmc
