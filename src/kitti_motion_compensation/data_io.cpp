#include "kitti_motion_compensation/data_io.hpp"

#include <fstream>
#include <iostream>

#include "kitti_motion_compensation/utils.hpp"

namespace kmc {

Time LoadTimeStamp(Path const timestamp_file, size_t const frame_id) {
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

  std::vector<std::string> const stamp_i_tokens{TokenizeString(stamp_line)};

  return Time(MmHhSsToSeconds(stamp_i_tokens[1]));
}

Oxts LoadOxts(Path const folder, size_t const frame_id) {
  // load the time stamp
  Path const timestamp_file(folder / Path("oxts/timestamps.txt"));
  Time const time{LoadTimeStamp(timestamp_file, frame_id)};

  // load the odometry
  Path const oxts_file(
      folder / Path("oxts/data/" + IdToZeroPaddedString(frame_id) + ".txt"));
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
  std::vector<std::string> const oxts_tokens{TokenizeString(oxts_line)};

  return Oxts{time, std::stod(oxts_tokens[8]), std::stod(oxts_tokens[9]),
              std::stod(oxts_tokens[10])};
}

Pointcloud LoadPointcloud(Path const pointcloud_file) {
  // this function is basically taken directly from the "devkit_raw_data" README

  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float *)malloc(num * sizeof(float));

  // pointers
  float *px = data + 0;
  float *py = data + 1;
  float *pz = data + 2;
  float *pr = data + 3;

  // load point cloud
  FILE *stream;
  stream = fopen(pointcloud_file.c_str(), "rb");
  num = fread(data, sizeof(float), num, stream) / 4;

  Pointcloud pointcloud = Eigen::MatrixX4d(num, 4);
  for (int32_t i = 0; i < num; i++) {
    pointcloud.row(i)(0) = *px;
    pointcloud.row(i)(1) = *py;
    pointcloud.row(i)(2) = *pz;
    pointcloud.row(i)(3) = *pr;

    px += 4;
    py += 4;
    pz += 4;
    pr += 4;
  }
  fclose(stream);

  return pointcloud;
}

LidarScan LoadLidarScan(Path const folder, size_t const frame_id) {
  // load the time stamps - start, middle and end of the scan - cameras are
  // triggered mechanically at the middle of the scan
  Path const start_timestamp_file(folder /
                                  Path("velodyne_points/timestamps_start.txt"));
  Path const middle_timestamp_file(folder /
                                   Path("velodyne_points/timestamps.txt"));
  Path const end_timestamp_file(folder /
                                Path("velodyne_points/timestamps_end.txt"));

  Time const start_time{LoadTimeStamp(start_timestamp_file, frame_id)};
  Time const middle_time{LoadTimeStamp(middle_timestamp_file, frame_id)};
  Time const end_time{LoadTimeStamp(end_timestamp_file, frame_id)};

  Path const pointcloud_file(
      folder /
      Path("velodyne_points/data/" + IdToZeroPaddedString(frame_id) + ".bin"));

  Pointcloud const pointcloud{LoadPointcloud(pointcloud_file)};

  return LidarScan{start_time, middle_time, end_time, pointcloud};
}

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images) {
  // load oxts
  Oxts odometry{LoadOxts(data_folder, frame_id)};

  // load scan
  LidarScan lidar_scan{LoadLidarScan(data_folder, frame_id)};

  // load images
  (void)load_images;

  return Frame(odometry, lidar_scan);
}

} // namespace kmc