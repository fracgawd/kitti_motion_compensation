#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images);

} // namespace kmc