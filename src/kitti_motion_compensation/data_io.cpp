#include "kitti_motion_compensation/data_io.hpp"

#include <fstream>
#include <iostream>

#include "kitti_motion_compensation/utils.hpp"

namespace kmc {

Time LoadTimeStamp(Path timestamp_file, size_t const frame_id) {
  (void)frame_id;
  std::ifstream is_timestamp(timestamp_file); // "is" = "input stream"

  std::string stamp_line;
  if (is_timestamp.is_open()) {
    // iterate until we get to the line holding the timestamp for <frame_id>
    for (size_t i{0}; i <= frame_id; ++i) {
      getline(is_timestamp, stamp_line);
    }
  } else {
    std::cout << "Failed to open timestamp file: " << timestamp_file << '\n';
    exit(0);
  }

  std::vector<std::string> stamp_i_tokens{TokenizeString(stamp_line)};

  return Time(MmHhSsToSeconds(stamp_i_tokens[1]));
}

Oxts LoadOxts(Path const folder, size_t const frame_id) {
  // load the time stamp
  Path timestamp_file(folder / Path("oxts/timestamps.txt"));
  Time const time{LoadTimeStamp(timestamp_file, frame_id)};

  // load the odometry
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

  // split the line up into the 30 individual values - values 8,9,10 are forward
  // velocity (vf), left velocity (vl) and upward velocity (vu) which is what we
  // need for motion compensation :)
  std::vector<std::string> oxts_tokens{TokenizeString(oxts_line)};

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