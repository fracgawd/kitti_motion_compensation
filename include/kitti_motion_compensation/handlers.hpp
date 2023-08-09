#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

std::size_t NumberOfFilesInDirectory(std::filesystem::path path);

void MotionCompensateRun(Path const run_folder);

} // namespace kmc
