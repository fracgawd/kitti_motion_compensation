#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Time LoadTimeStamp(Path timestamp_file, size_t const frame_id);

Oxts LoadOxts(Path folder, size_t frame_id);

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images = false);

} // namespace kmc