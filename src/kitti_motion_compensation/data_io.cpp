#include "kitti_motion_compensation/data_io.hpp"

namespace kmc {

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images) {
  (void)data_folder;
  (void)frame_id;
  (void)load_images;

  return Frame(Oxts(), LidarCloud());
}

} // namespace kmc