#include "kitti_motion_compensation/data_io.hpp"

#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>

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
  // triggered at the middle of the scan
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

Image LoadImage(Path const folder, std::string const camera,
                size_t const frame_id) {
  // Note: interesting that for the test sequence image_00/01 and image_02/03
  // are seperated by a consitent 0.01 seconds to 0.005 second. For example for
  // frame_id=0 the timestamps are as follows:
  //
  //  image_00: 47072.351950336
  //  image_01: 47072.351951104
  //  image_02: 47072.345808896
  //  image_03: 47072.345323776
  //
  // TODO(jack): investigate if assuming all images were at same timestamp
  // introduces bias

  Path const timestamp_file(folder / Path(camera + "/timestamps.txt"));
  Time const time{LoadTimeStamp(timestamp_file, frame_id)};

  Path const image_file(folder / Path(camera + "/data/" +
                                      IdToZeroPaddedString(frame_id) + ".png"));
  cv::Mat image;
  if (camera == "image_00" or camera == "image_01") {
    image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
  } else {
    image = cv::imread(image_file, cv::IMREAD_COLOR);
  }

  if (image.empty()) {
    std::cout << "Failed to load image: " << image_file << '\n';
    exit(0);
  }

  return Image{time, image};
}

Images LoadImages(Path const folder, size_t const frame_id) {
  Image const img_00{LoadImage(folder, "image_00", frame_id)};
  Image const img_01{LoadImage(folder, "image_01", frame_id)};
  Image const img_02{LoadImage(folder, "image_02", frame_id)};
  Image const img_03{LoadImage(folder, "image_03", frame_id)};

  return Images{img_00, img_01, img_02, img_03};
}

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images) {
  // load oxts
  Oxts const odometry{LoadOxts(data_folder, frame_id)};

  // load scan
  LidarScan const lidar_scan{LoadLidarScan(data_folder, frame_id)};

  // load images if requested and return
  if (load_images) {
    Images const images{LoadImages(data_folder, frame_id)};
    return Frame(odometry, lidar_scan, images);
  } else {
    return Frame(odometry, lidar_scan);
  }
}

void WritePointcloud(Path const data_folder, size_t const frame_id,
                     Pointcloud const &pointcloud) {
  // I tried to write this function like the LoadPointcloud function but in
  // reverse. It didn't work and I ended up stumbling around the web for a
  // little bit. Current code is modeled after
  // https://stackoverflow.com/questions/30923685/writing-floats-to-a-binary-file-in-c-equivalent-of-javas-dataoutputstream-w

  Path const pointcloud_file(data_folder /
                             Path(IdToZeroPaddedString(frame_id) + ".bin"));

  std::ofstream out;
  out.open(pointcloud_file, std::ios::out | std::ios::binary);

  int32_t num_points = pointcloud.rows();
  for (int32_t i = 0; i < num_points; i++) {
    auto const point = pointcloud.row(i);
    float x{static_cast<float>(point(0))};
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
    x = static_cast<float>(point(1));
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
    x = static_cast<float>(point(2));
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
    x = static_cast<float>(point(3));
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
  }
  out.close();
}

} // namespace kmc

namespace kmc::viz {

namespace fs = std::filesystem;

// Eigen::Vector2d S;
// Eigen::Matrix3d K;
// Eigen::Matrix<double, 5, 1> D;
// Eigen::Matrix3d R;
// Eigen::Vector3d T;
// Eigen::Vector2d S_rect;
// Eigen::Matrix3d R_rect;
CameraCalibration CalibrationLinesToCalibration(
    std::vector<std::string> const calibration_lines) {
  // this is really an unfortunate function but we need to convert the
  // calibration txt files into eigen matrics somehow :) If someone has a
  // better idea please let me know.

  // S
  std::vector<std::string> S_tokens{kmc::TokenizeString(calibration_lines[0])};

  Eigen::Vector2d S;
  S << std::stod(S_tokens[1]), std::stod(S_tokens[2]);

  // K
  std::vector<std::string> K_tokens{kmc::TokenizeString(calibration_lines[1])};

  Eigen::Matrix3d K;
  K << std::stod(K_tokens[1]), std::stod(K_tokens[2]), std::stod(K_tokens[3]),
      std::stod(K_tokens[4]), std::stod(K_tokens[5]), std::stod(K_tokens[6]),
      std::stod(K_tokens[7]), std::stod(K_tokens[8]), std::stod(K_tokens[9]);

  // D
  std::vector<std::string> D_tokens{kmc::TokenizeString(calibration_lines[2])};

  Eigen::Matrix<double, 5, 1> D;
  D << std::stod(D_tokens[1]), std::stod(D_tokens[2]), std::stod(D_tokens[3]),
      std::stod(D_tokens[4]), std::stod(D_tokens[5]);

  // R
  std::vector<std::string> R_tokens{kmc::TokenizeString(calibration_lines[3])};

  Eigen::Matrix3d R;
  R << std::stod(R_tokens[1]), std::stod(R_tokens[2]), std::stod(R_tokens[3]),
      std::stod(R_tokens[4]), std::stod(R_tokens[5]), std::stod(R_tokens[6]),
      std::stod(R_tokens[7]), std::stod(R_tokens[8]), std::stod(R_tokens[9]);

  // T
  std::vector<std::string> T_tokens{kmc::TokenizeString(calibration_lines[4])};

  Eigen::Vector3d T;
  T << std::stod(T_tokens[1]), std::stod(T_tokens[2]), std::stod(T_tokens[3]);

  // S_rect
  std::vector<std::string> S_rect_tokens{
      kmc::TokenizeString(calibration_lines[5])};

  Eigen::Vector2d S_rect;
  S_rect << std::stod(S_rect_tokens[1]), std::stod(S_rect_tokens[2]);

  // R_rect
  std::vector<std::string> R_rect_tokens{
      kmc::TokenizeString(calibration_lines[6])};

  Eigen::Matrix3d R_rect;
  R_rect << std::stod(R_rect_tokens[1]), std::stod(R_rect_tokens[2]),
      std::stod(R_rect_tokens[3]), std::stod(R_rect_tokens[4]),
      std::stod(R_rect_tokens[5]), std::stod(R_rect_tokens[6]),
      std::stod(R_rect_tokens[7]), std::stod(R_rect_tokens[8]),
      std::stod(R_rect_tokens[9]);

  // P_rect
  std::vector<std::string> P_rect_tokens{
      kmc::TokenizeString(calibration_lines[7])};

  P P_rect;
  P_rect << std::stod(P_rect_tokens[1]), std::stod(P_rect_tokens[2]),
      std::stod(P_rect_tokens[3]), std::stod(P_rect_tokens[4]),
      std::stod(P_rect_tokens[5]), std::stod(P_rect_tokens[6]),
      std::stod(P_rect_tokens[7]), std::stod(P_rect_tokens[8]),
      std::stod(P_rect_tokens[9]), std::stod(P_rect_tokens[10]),
      std::stod(P_rect_tokens[11]), std::stod(P_rect_tokens[12]);

  return CameraCalibration{S, K, D, R, T, S_rect, R_rect, P_rect};
}

CameraCalibrations LoadCameraCalibrations(kmc::Path const data_folder) {
  kmc::Path const calibration_file(data_folder /
                                   kmc::Path("calib_cam_to_cam.txt"));
  std::ifstream calibration_is(calibration_file);

  if (not calibration_is.is_open()) {
    std::cout << "Failed to open camera calibration file: " << calibration_file
              << '\n';
    exit(0);
  }

  // throw out the first two lines that contain meta information
  std::string calibration_line;
  std::getline(calibration_is, calibration_line);
  std::getline(calibration_is, calibration_line);

  // load the four camera calibrations - each one has eight lines
  std::vector<CameraCalibration> camera_calibrations;
  for (size_t n{0}; n < 4; ++n) {
    std::vector<std::string> calibration_lines;

    for (size_t i{0}; i < 8; ++i) {
      std::getline(calibration_is, calibration_line);
      calibration_lines.push_back(calibration_line);
    }

    CameraCalibration camera_calibration{
        CalibrationLinesToCalibration(calibration_lines)};
    camera_calibrations.push_back(camera_calibration);
  }

  calibration_is.close();

  return CameraCalibrations{camera_calibrations[0], camera_calibrations[1],
                            camera_calibrations[2], camera_calibrations[3]};
}

Eigen::Affine3d LoadLidarExtrinsics(kmc::Path const data_folder) {
  kmc::Path const calibration_file(data_folder /
                                   kmc::Path("calib_velo_to_cam.txt"));
  std::ifstream calibration_is(calibration_file);

  if (not calibration_is.is_open()) {
    std::cout << "Failed to open camera calibration file: " << calibration_file
              << '\n';
    exit(0);
  }

  // throw out the first line that contains meta information
  std::string calibration_line;
  std::getline(calibration_is, calibration_line);

  // R
  std::getline(calibration_is, calibration_line);
  std::vector<std::string> R_tokens{kmc::TokenizeString(calibration_line)};

  Eigen::Matrix3d R;
  R << std::stod(R_tokens[1]), std::stod(R_tokens[2]), std::stod(R_tokens[3]),
      std::stod(R_tokens[4]), std::stod(R_tokens[5]), std::stod(R_tokens[6]),
      std::stod(R_tokens[7]), std::stod(R_tokens[8]), std::stod(R_tokens[9]);

  // T
  std::getline(calibration_is, calibration_line);
  std::vector<std::string> T_tokens{kmc::TokenizeString(calibration_line)};

  Eigen::Vector3d T;
  T << std::stod(T_tokens[1]), std::stod(T_tokens[2]), std::stod(T_tokens[3]);

  // transform from lidar frame to camera_00
  Eigen::Affine3d tf_c00_lo{Eigen::Affine3d::Identity()};

  tf_c00_lo = R * tf_c00_lo;
  tf_c00_lo.translation() = T;

  return tf_c00_lo;
}

void MakeOutputImageFolders(Path const output_folder) {
  // TODO(jack): eliminate copy and pasted code

  Path const image_00_folder{output_folder / Path("image_00")};
  if (not fs::is_directory(image_00_folder) ||
      not fs::exists(image_00_folder)) {
    fs::create_directory(image_00_folder);
  } else {
    std::cout << "The directory: " << image_00_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }

  Path const image_01_folder{output_folder / Path("image_01")};
  if (not fs::is_directory(image_01_folder) ||
      not fs::exists(image_01_folder)) {
    fs::create_directory(image_01_folder);
  } else {
    std::cout << "The directory: " << image_01_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }

  Path const image_02_folder{output_folder / Path("image_02")};
  if (not fs::is_directory(image_02_folder) ||
      not fs::exists(image_02_folder)) {
    fs::create_directory(image_02_folder);
  } else {
    std::cout << "The directory: " << image_02_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }

  Path const image_03_folder{output_folder / Path("image_03")};
  if (not fs::is_directory(image_03_folder) ||
      not fs::exists(image_03_folder)) {
    fs::create_directory(image_03_folder);
  } else {
    std::cout << "The directory: " << image_03_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }
}

void SaveImagesOnTopOfEachother(Images const &top_imgs,
                                Images const &bottom_imgs,
                                size_t const frame_id,
                                Path const output_folder) {
  // Note that this function depends on the fact that the MakeOutputImageFolders
  // was called and created the directories we need to write to in the
  // output_folder

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_00.image, bottom_imgs.image_00.image,
      output_folder /
          Path("image_00/" + IdToZeroPaddedString(frame_id) + ".png"));

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_01.image, bottom_imgs.image_01.image,
      output_folder /
          Path("image_01/" + IdToZeroPaddedString(frame_id) + ".png"));

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_02.image, bottom_imgs.image_02.image,
      output_folder /
          Path("image_02/" + IdToZeroPaddedString(frame_id) + ".png"));

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_03.image, bottom_imgs.image_03.image,
      output_folder /
          Path("image_03/" + IdToZeroPaddedString(frame_id) + ".png"));

  return;
}

} // namespace kmc::viz