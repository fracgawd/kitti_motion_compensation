#include "kitti_motion_compensation/handlers.hpp"

#include <algorithm>
#include <filesystem>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"

namespace kmc {

namespace fs = std::filesystem;

std::size_t NumberOfFilesInDirectory(fs::path path) {
  return (std::size_t)std::distance(fs::directory_iterator{path},
                                    fs::directory_iterator{});
}

void MotionCompensateRun(Path const run_folder) {
  Path const velodyne_folder{run_folder / Path{"velodyne_points"}};

  // we need to figure out how many frames there are in the run, so check the
  // length of the velodyne_points data folder. We could have checked any data
  // folder but this one was close by
  size_t number_of_frames{
      NumberOfFilesInDirectory(velodyne_folder / Path("data"))};

  // we need to create the folder we want to write the data to because it
  // wouldn't be nice to overwrite the original pointclouds :)
  Path const data_folder{velodyne_folder / Path("data_motion_compensated")};
  if (not fs::is_directory(data_folder) || not fs::exists(data_folder)) {
    fs::create_directory(data_folder);
  }

  for (size_t i{0}; i < number_of_frames; ++i) {
    Frame const frame{LoadSingleFrame(run_folder, i)};

    // TODO(jack): it logically makes most sense to motion compensate each cloud
    // to each respective image time. We just choose the middle of the scan
    // cause that is when at least the lidar triggers the cameras. Not perfect
    // but close.
    Time const requested_time{frame.scan_.stamp_middle};
    // TODO(jack): transfer intensities
    Pointcloud const motion_compensated_pointcloud{
        MotionCompensate(frame, requested_time)};

    WritePointcloud(data_folder, i, motion_compensated_pointcloud);
  }
}

} // namespace kmc
