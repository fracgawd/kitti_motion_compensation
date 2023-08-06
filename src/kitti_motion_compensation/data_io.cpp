#include "kitti_motion_compensation/data_io.hpp"

#include <fstream>
#include <iostream>

#include "kitti_motion_compensation/utils.hpp"

namespace kmc {

Oxts LoadOxts(Path const folder, size_t const frame_id) {

  // load frame time stamp
  Path timestamp_file(folder / Path("oxts/timestamps.txt"));
  std::cout << timestamp_file << std::endl;

  Time const time;

  // load frame odometry
  Path oxts_file(folder /
                 Path("oxts/data/" + IdToZeroPaddedString(frame_id) + ".txt"));
  std::ifstream is_oxts(oxts_file); // "is" = "input stream"

  std::string oxts_line;
  if (is_oxts.is_open()) {
    std::getline(is_oxts, oxts_line);
    is_oxts.close();
  } else {
    std::cout << "Failed to open oxts file: " << oxts_file << '\n';
    exit(0);
  }

  auto oxts_tokens{TokenizeString(oxts_line)};

  return Oxts{time, std::stod(oxts_tokens[8]), std::stod(oxts_tokens[9]),
              std::stod(oxts_tokens[10])};
}

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images) {
  (void)load_images;

  // load oxts
  Oxts odometry{LoadOxts(data_folder, frame_id)};

  // load pointcloud

  // load images

  return Frame(odometry, LidarCloud());
}

} // namespace kmc