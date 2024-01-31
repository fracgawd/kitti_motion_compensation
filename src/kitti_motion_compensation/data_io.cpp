#include "kitti_motion_compensation/data_io.hpp"

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "kitti_motion_compensation/data_types.hpp"
#include "kitti_motion_compensation/lie_algebra.hpp"
#include "kitti_motion_compensation/timestamp_mocking.hpp"
#include "kitti_motion_compensation/trajectory_interpolation.hpp"
#include "kitti_motion_compensation/utils.hpp"

namespace kmc {

Time LoadTimeStamp(Path const timestamp_file, size_t const frame_id) {
  std::ifstream is_timestamp(timestamp_file);  // "is" = "input stream"

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

std::optional<Oxts> LoadOxts(Path const folder, size_t const frame_id) {
  // WARN(jack): This function is really not great. The problem is that it essentially decides if the `frame_id` is
  // valid based on the ability of the oxts input stream to be opened. This hopefully will not work when the `frame_id`
  // requests a frame that is not in the sequence. This really does happen all the time, and it works, but it is a
  // brittle solution.
  // An example of this brittleness is that the timestamp loading part had to be moved after the odometry loading part.
  // The reason for this was that we need the early std::nullopt return to protect the `LoadTimeStamp` function from
  // getting an invalid `frame_id`, because that function is not hardened against invalid frame ids. The real answer to
  // the problem I am talking about to much is to harden all functions against invalid id access, but this would add a
  // lot of boilerplate and complexity. The KITTI is so well structured, that I will provide what security I can, but I
  // also need to count on the fact that people will use the functins within some limit of sanity.

  // load the odometry
  Path const oxts_file(folder / Path("oxts/data/" + IdToZeroPaddedString(frame_id) + ".txt"));
  std::ifstream is_oxts(oxts_file);  // "is" = "input stream"

  std::string oxts_line;
  if (is_oxts.is_open()) {
    std::getline(is_oxts, oxts_line);
    is_oxts.close();
  } else {
    std::cerr << "Failed to open oxts file: " << oxts_file << '\n';
    return std::nullopt;
  }

  // load the time stamp
  Path const timestamp_file(folder / Path("oxts/timestamps.txt"));
  Time const time{LoadTimeStamp(timestamp_file, frame_id)};

  std::vector<std::string> const oxts_tokens{TokenizeString(oxts_line)};

  return Oxts{time,
              std::stod(oxts_tokens[0]),
              std::stod(oxts_tokens[1]),
              std::stod(oxts_tokens[2]),
              std::stod(oxts_tokens[3]),
              std::stod(oxts_tokens[4]),
              std::stod(oxts_tokens[5]),
              std::stod(oxts_tokens[8]),
              std::stod(oxts_tokens[9]),
              std::stod(oxts_tokens[10])};
}

Eigen::Affine3d OxtsToPose(Oxts const &odometry, double const scale) {
  // earth radius (approx.) in meters
  double const earth_radius{6378137.0};

  // mercator projection to get position
  double const tx{scale * earth_radius * M_PI * odometry.lon / 180.0};
  double const ty{scale * earth_radius * std::log(std::tan(M_PI * (90.0 + odometry.lat) / 360.0))};
  double const tz{odometry.alt};

  // use the Euler angles to get the rotation matrix
  Eigen::Matrix3d const R{Eigen::AngleAxisd(odometry.yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(odometry.pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(odometry.roll, Eigen::Vector3d::UnitX())};

  // combine the translation and rotation into a homogeneous transform
  Eigen::Affine3d pose{Eigen::Affine3d::Identity()};
  pose = R * pose;
  pose.translation() << tx, ty, tz;

  return pose;
}

///////////////////////////////////////////////////////////////

// Each scan is about 2MB. Each point consists of 4 floats. Each float is of course 4 bytes. Therefore in each scan we
// can calculate there are about 125,000 points. This is roughly the number points actually found in the pointcloud.
//
// To be safe, lets build in a safety factor of two and allocate 4MB of memory for the loader, which corresponds to
// 250,000 points.
KittiPclLoader::KittiPclLoader() : data_{new float[pcl_buffer_size]} {}

KittiPclLoader::~KittiPclLoader() { delete[] data_; }

std::tuple<Pointcloud, VectorXd> KittiPclLoader::LoadPointcloud(Path const &file) {
  std::ifstream file_stream{file, std::ios::in | std::ios::binary | std::ios::ate};
  if (not file_stream.is_open()) {
    throw std::runtime_error("Unable to open requested KITTI pointcloud binary file: " + std::string{file});
  }

  int64_t const num_bytes_loaded{file_stream.tellg()};
  if ((num_bytes_loaded == -1) or ((num_bytes_loaded % 4) != 0)) {
    throw std::runtime_error("Opened KITTI pointcloud binary file is incorrectly formatted: " + std::string{file});
  }

  size_t const num_points{static_cast<size_t>(num_bytes_loaded) / 16};  // each point (xyzi) is 16 bytes

  file_stream.seekg(0, std::ios::beg);
  file_stream.read(reinterpret_cast<char *>(data_), num_bytes_loaded);
  file_stream.close();

  // Because the the value of `data_` itself never changes, we could just intitilaize the `data_pointers` once and write
  // a `Reset` method to set them back to the to start after each new pointcloud is processed. But I think it is also
  // appropriate to just locally initialize it each time here, rather than making sure to call `Reset()` after every
  // pointcloud is loaded and processed.
  PclPointers data_pointers{data_};

  Pointcloud pointcloud{Eigen::MatrixX4d(num_points, 4)};
  VectorXd intensities{VectorXd(num_points)};
  for (size_t i{0}; i < num_points; ++i) {
    pointcloud.row(i)(0) = *(data_pointers.x);
    pointcloud.row(i)(1) = *(data_pointers.y);
    pointcloud.row(i)(2) = *(data_pointers.z);
    pointcloud.row(i)(3) = 1.0;  // homogenous component

    intensities(i) = *(data_pointers.i);

    data_pointers.Increment();
  }

  return {pointcloud, intensities};
}

////////////////////////////////////////////

LidarScan LoadLidarScan(Path const folder, size_t const frame_id) {
  // load the time stamps - start, middle and end of the scan - cameras are
  // triggered at the middle of the scan
  Path const start_timestamp_file(folder / Path("velodyne_points/timestamps_start.txt"));
  Path const middle_timestamp_file(folder / Path("velodyne_points/timestamps.txt"));
  Path const end_timestamp_file(folder / Path("velodyne_points/timestamps_end.txt"));

  Time const start_time{LoadTimeStamp(start_timestamp_file, frame_id)};
  Time const middle_time{LoadTimeStamp(middle_timestamp_file, frame_id)};
  Time const end_time{LoadTimeStamp(end_timestamp_file, frame_id)};

  // Ok honestly this is not the original vision for how I planned to use the `KittiPclLoader`. I had a the vision that
  // we would create it once, allocate it's resources, and then loade all the pointclouds using that same resource using
  // the `LoadPointcloud()` method. But right now, because the way the loader is designed to work exclusively in a
  // "frame wise" functional fashion, with no state, we actually need to construct the handler object each time.
  //
  // TODO(jack): design the loading method so that `KittiPclLoader` doesn't need to be reconstructed every time.
  KittiPclLoader pcl_loader_handler;
  Path const pointcloud_file(folder / Path("velodyne_points/data/" + IdToZeroPaddedString(frame_id) + ".bin"));
  auto const [pointcloud, intensities] = pcl_loader_handler.LoadPointcloud(pointcloud_file);

  VectorXd const timestamps{GetPseudoTimeStamps(pointcloud, start_time, end_time)};

  return LidarScan{start_time, middle_time, end_time, pointcloud, intensities, timestamps};
}

Eigen::Affine3d LoadLidarExtrinsics(kmc::Path const data_folder, bool const to_cam) {
  kmc::Path calibration_file;
  if (to_cam) {
    calibration_file = (data_folder / kmc::Path("calib_velo_to_cam.txt"));
  } else {
    calibration_file = (data_folder / kmc::Path("calib_imu_to_velo.txt"));
  }

  std::ifstream calibration_is(calibration_file);

  if (not calibration_is.is_open()) {
    std::cout << "Failed to open camera calibration file: " << calibration_file << '\n';
    exit(0);
  }

  // throw out the first line that contains meta information
  std::string calibration_line;
  std::getline(calibration_is, calibration_line);

  // R
  std::getline(calibration_is, calibration_line);
  std::vector<std::string> R_tokens{kmc::TokenizeString(calibration_line)};

  Eigen::Matrix3d R;
  R << std::stod(R_tokens[1]), std::stod(R_tokens[2]), std::stod(R_tokens[3]), std::stod(R_tokens[4]),
      std::stod(R_tokens[5]), std::stod(R_tokens[6]), std::stod(R_tokens[7]), std::stod(R_tokens[8]),
      std::stod(R_tokens[9]);

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

Image LoadImage(Path const folder, std::string const camera, size_t const frame_id) {
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

  Path const image_file(folder / Path(camera + "/data/" + IdToZeroPaddedString(frame_id) + ".png"));
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

Frame MakeFrame(kmc::Oxts const &odometry_n_m_1, kmc::Oxts const &odometry_n, kmc::Oxts const &odometry_n_p_1,
                kmc::LidarScan const &lidar_scan, std::optional<kmc::Images> const camera_images) {
  // odometry_n_m_1 - oxts packet of the previous frame (n_m_1 = "n minus 1")
  // odometry_n - oxts of the current frame (n), usually happens at around the middle of the scan
  // odometry_n_p_1 - oxts packet of the next frame (n_p_1 = "n plus 1")

  // Because we do not directly get the pose of the sensor at the start and end of the scan we need to interpolate in
  // between the nearest odometry measurements. For the start of the scan that will be "n-1" and "n" and for the end of
  // the scan that will be "n" and "n+1".

  Affine3d const scan_start_pose{InterpolateFramePose(odometry_n_m_1, odometry_n, lidar_scan.stamp_start)};
  Affine3d const scan_end_pose{InterpolateFramePose(odometry_n, odometry_n_p_1, lidar_scan.stamp_end)};

  return Frame(scan_start_pose, scan_end_pose, lidar_scan, camera_images);
}

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id, bool const load_images) {
  // TODO(jack): what about const with std::optional
  // TODO(jack): are the statements about which part of the cloud can and cannot be motion compensated correct?
  // TODO(jack): refactor the odometry loading thing into its own helper function

  std::array<Oxts, 3> odometry;
  for (int i{-1}; i <= 1; ++i) {
    // TODO(jack): use fancy "if with initializtion"
    std::optional<Oxts> odometry_i{LoadOxts(data_folder, frame_id + i)};

    if (odometry_i.has_value()) {
      // For all non-edge frames (1 -> n-1), this is the only condition that should execute.
      odometry[i + 1] = odometry_i.value();
    } else if (-1 == i) {
      // This is the failure that should only happen when we attempt to load the first frame of a sequence. Because
      // there is no n-1 Oxts packet, we just load the first Oxts packet of the sequence and fill it with that. This
      // means that the first half of the scan, before the first Oxts packet was recieved, cannot be motion compensated.
      std::optional<Oxts> odometry_0{LoadOxts(data_folder, frame_id + (i + 1))};
      if (not odometry_0.has_value()) {
        std::cerr << "Could not load replacement edge frame (frame(-1)) Oxts packet :()" << std::endl;
        continue;  // WARN(jack); is continue the right behavior?
      }
      odometry[i + 1] = odometry_0.value();
    } else if (1 == i) {
      // This is similar to the failure before, except that now we are at the last frame of the sequence, and can only
      // compensate the first half.
      std::optional<Oxts> odometry_n{LoadOxts(data_folder, frame_id + (i - 1))};
      if (not odometry_n.has_value()) {
        std::cerr << "Could not load replacement edge frame (frame(n+1)) Oxts packet :()" << std::endl;
        continue;  // WARN(jack); is continue the right behavior?
      }
      odometry[i + 1] = odometry_n.value();
    }
  }

  // load scan
  LidarScan const lidar_scan{LoadLidarScan(data_folder, frame_id)};

  // load images if requested and return
  if (load_images) {
    Images const images{LoadImages(data_folder, frame_id)};
    return MakeFrame(odometry[0], odometry[1], odometry[2], lidar_scan, images);
  } else {
    return MakeFrame(odometry[0], odometry[1], odometry[2], lidar_scan);
  }
}

void WritePointcloud(Path const data_folder, size_t const frame_id, Pointcloud const &pointcloud,
                     VectorXd const &intensities) {
  // I tried to write this function like the LoadPointcloud function but in
  // reverse. It didn't work and I ended up stumbling around the web for a
  // little bit. Current code is modeled after
  // https://stackoverflow.com/questions/30923685/writing-floats-to-a-binary-file-in-c-equivalent-of-javas-dataoutputstream-w

  Path const pointcloud_file(data_folder / Path(IdToZeroPaddedString(frame_id) + ".bin"));

  std::ofstream out;
  out.open(pointcloud_file, std::ios::out | std::ios::binary);

  int32_t const num_points = pointcloud.rows();
  for (int32_t i = 0; i < num_points; i++) {
    auto const point = pointcloud.row(i);
    float x{static_cast<float>(point(0))};
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
    x = static_cast<float>(point(1));
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
    x = static_cast<float>(point(2));
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
    x = static_cast<float>(intensities(i));
    out.write(reinterpret_cast<const char *>(&x), sizeof(float));
  }
  out.close();
}

}  // namespace kmc

namespace kmc::viz {

namespace fs = std::filesystem;

// Eigen::Vector2d S;
// Eigen::Matrix3d K;
// Eigen::Matrix<double, 5, 1> D;
// Eigen::Matrix3d R;
// Eigen::Vector3d T;
// Eigen::Vector2d S_rect;
// Eigen::Matrix3d R_rect;
CameraCalibration CalibrationLinesToCalibration(std::vector<std::string> const calibration_lines) {
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
  K << std::stod(K_tokens[1]), std::stod(K_tokens[2]), std::stod(K_tokens[3]), std::stod(K_tokens[4]),
      std::stod(K_tokens[5]), std::stod(K_tokens[6]), std::stod(K_tokens[7]), std::stod(K_tokens[8]),
      std::stod(K_tokens[9]);

  // D
  std::vector<std::string> D_tokens{kmc::TokenizeString(calibration_lines[2])};

  Eigen::Matrix<double, 5, 1> D;
  D << std::stod(D_tokens[1]), std::stod(D_tokens[2]), std::stod(D_tokens[3]), std::stod(D_tokens[4]),
      std::stod(D_tokens[5]);

  // R
  std::vector<std::string> R_tokens{kmc::TokenizeString(calibration_lines[3])};

  Eigen::Matrix3d R;
  R << std::stod(R_tokens[1]), std::stod(R_tokens[2]), std::stod(R_tokens[3]), std::stod(R_tokens[4]),
      std::stod(R_tokens[5]), std::stod(R_tokens[6]), std::stod(R_tokens[7]), std::stod(R_tokens[8]),
      std::stod(R_tokens[9]);

  // T
  std::vector<std::string> T_tokens{kmc::TokenizeString(calibration_lines[4])};

  Eigen::Vector3d T;
  T << std::stod(T_tokens[1]), std::stod(T_tokens[2]), std::stod(T_tokens[3]);

  // S_rect
  std::vector<std::string> S_rect_tokens{kmc::TokenizeString(calibration_lines[5])};

  Eigen::Vector2d S_rect;
  S_rect << std::stod(S_rect_tokens[1]), std::stod(S_rect_tokens[2]);

  // R_rect
  std::vector<std::string> R_rect_tokens{kmc::TokenizeString(calibration_lines[6])};

  Eigen::Matrix3d R_rect;
  R_rect << std::stod(R_rect_tokens[1]), std::stod(R_rect_tokens[2]), std::stod(R_rect_tokens[3]),
      std::stod(R_rect_tokens[4]), std::stod(R_rect_tokens[5]), std::stod(R_rect_tokens[6]),
      std::stod(R_rect_tokens[7]), std::stod(R_rect_tokens[8]), std::stod(R_rect_tokens[9]);

  // P_rect
  std::vector<std::string> P_rect_tokens{kmc::TokenizeString(calibration_lines[7])};

  P P_rect;
  P_rect << std::stod(P_rect_tokens[1]), std::stod(P_rect_tokens[2]), std::stod(P_rect_tokens[3]),
      std::stod(P_rect_tokens[4]), std::stod(P_rect_tokens[5]), std::stod(P_rect_tokens[6]),
      std::stod(P_rect_tokens[7]), std::stod(P_rect_tokens[8]), std::stod(P_rect_tokens[9]),
      std::stod(P_rect_tokens[10]), std::stod(P_rect_tokens[11]), std::stod(P_rect_tokens[12]);

  return CameraCalibration{S, K, D, R, T, S_rect, R_rect, P_rect};
}

CameraCalibrations LoadCameraCalibrations(kmc::Path const data_folder) {
  kmc::Path const calibration_file(data_folder / kmc::Path("calib_cam_to_cam.txt"));
  std::ifstream calibration_is(calibration_file);

  if (not calibration_is.is_open()) {
    std::cout << "Failed to open camera calibration file: " << calibration_file << '\n';
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

    CameraCalibration camera_calibration{CalibrationLinesToCalibration(calibration_lines)};
    camera_calibrations.push_back(camera_calibration);
  }

  calibration_is.close();

  return CameraCalibrations{camera_calibrations[0], camera_calibrations[1], camera_calibrations[2],
                            camera_calibrations[3]};
}

void MakeOutputImageFolders(Path const output_folder) {
  // TODO(jack): eliminate copy and pasted code

  Path const image_00_folder{output_folder / Path("image_00")};
  if (not fs::is_directory(image_00_folder) || not fs::exists(image_00_folder)) {
    fs::create_directory(image_00_folder);
    std::cout << "Creating directory: " << image_00_folder << std::endl;
  } else {
    std::cout << "The directory: " << image_00_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }

  Path const image_01_folder{output_folder / Path("image_01")};
  if (not fs::is_directory(image_01_folder) || not fs::exists(image_01_folder)) {
    fs::create_directory(image_01_folder);
    std::cout << "Creating directory: " << image_01_folder << std::endl;
  } else {
    std::cout << "The directory: " << image_01_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }

  Path const image_02_folder{output_folder / Path("image_02")};
  if (not fs::is_directory(image_02_folder) || not fs::exists(image_02_folder)) {
    fs::create_directory(image_02_folder);
    std::cout << "Creating directory: " << image_02_folder << std::endl;
  } else {
    std::cout << "The directory: " << image_02_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }

  Path const image_03_folder{output_folder / Path("image_03")};
  if (not fs::is_directory(image_03_folder) || not fs::exists(image_03_folder)) {
    fs::create_directory(image_03_folder);
    std::cout << "Creating directory: " << image_03_folder << std::endl;
  } else {
    std::cout << "The directory: " << image_03_folder
              << " already exists and you are probbaly about to overwrite the "
                 "images you have in there "
              << std::endl;
  }
}

void SaveImagesOnTopOfEachother(Images const &top_imgs, Images const &bottom_imgs, size_t const frame_id,
                                Path const output_folder) {
  // Note that this function depends on the fact that the MakeOutputImageFolders
  // was called and created the directories we need to write to in the
  // output_folder

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_00.image, bottom_imgs.image_00.image,
      output_folder / Path("image_00/" + IdToZeroPaddedString(frame_id) + ".png"));

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_01.image, bottom_imgs.image_01.image,
      output_folder / Path("image_01/" + IdToZeroPaddedString(frame_id) + ".png"));

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_02.image, bottom_imgs.image_02.image,
      output_folder / Path("image_02/" + IdToZeroPaddedString(frame_id) + ".png"));

  viz::utils::StackAndProcessProjectionImagePair(
      top_imgs.image_03.image, bottom_imgs.image_03.image,
      output_folder / Path("image_03/" + IdToZeroPaddedString(frame_id) + ".png"));

  return;
}

}  // namespace kmc::viz
