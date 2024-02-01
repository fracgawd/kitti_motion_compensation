#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Pointcloud MotionCompensateFrame(Frame const& frame, Time const requested_time);

}  // namespace kmc
