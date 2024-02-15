#pragma once

#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/trajectory_interpolation.hpp"

namespace kmc {

using TrajectoryInterpolator = trajectory_interpolation::TrajectoryInterpolator;

Vector4d MotionCompensatePoint(TrajectoryInterpolator const& trajectory_interpolator, Time const point_stamp,
                               Vector4d const& point, Time const requested_time);

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time);

}  // namespace kmc
