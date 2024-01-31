#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Time LoadTimeStamp(Path const timestamp_file, size_t const frame_id);

std::optional<Oxts> LoadOxts(Path const folder, size_t const frame_id);

// Implementation adopted from https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
Eigen::Affine3d OxtsToPose(Oxts const &odometry, double const scale = 1.0);

constexpr size_t float_entries_per_point{4};                         // [x y z intensity]_n
constexpr size_t pcl_buffer_size{250000 * float_entries_per_point};  // 250,000 points includes a 2x safety factor

class KittiPclLoader {
 private:
  struct PclPointers {
    // This is a handler to make iterating over the raw data array a little easier.
    //
    // The raw data array looks like:
    //
    //   x_0 y_0 z_0 i_0   # Each point is represented by four floats (16 bytes total)
    //   x_1 y_1 z_1 i_1
    //   ...
    //   x_n y_n z_n i_n
    //
    // But of course the data is actually stored/loaded as a 1D array and actually looks more like:
    //
    //    x_0 y_0 z_0 i_0 x_1 y_1 z_1 i_1 ... x_n y_n z_n i_n
    //
    // Therefore we have this handy little `PclPointers` handler class to make iterating over the data a little easier
    // and more stuctured. When the handler is initialized with the pointer to `data` the pointers will looks like:
    //
    //   x_0 y_0 z_0 i_0 x_1 y_1 z_1 i_1 ... x_n y_n z_n i_n
    //
    //   ^   ^   ^   ^
    //   x   y   z   i
    //
    // And a call to the `Inrement()` method will move the pointer set down the array by the stride
    // `float_entries_per_point`:
    //
    //   x_0 y_0 z_0 i_0 x_1 y_1 z_1 i_1 ... x_n y_n z_n i_n
    //
    //                   ^   ^   ^   ^
    //                   x   y   z   i

    PclPointers(float const *const data) : x{data}, y{data + 1}, z{data + 2}, i{data + 3} {}

    void Increment() {
      // increment pointers by the "stride"
      x = x + float_entries_per_point;
      y = y + float_entries_per_point;
      z = z + float_entries_per_point;
      i = i + float_entries_per_point;
    }

    // Mutable pointers to const data. The const here masks the values we are reading and prevents us from accidentally
    // editing them while using the handler. But of course the pointer itself is not const, because we need to increment
    // them to iterate over the data array :)
    float const *x;
    float const *y;
    float const *z;
    float const *i;
  };

 public:
  KittiPclLoader();

  ~KittiPclLoader();

  KittiPclLoader(KittiPclLoader const &other) = delete;  // copy constructor

  KittiPclLoader &operator=(KittiPclLoader const &other) = delete;  // copy assignment

  std::tuple<Pointcloud, VectorXd> LoadPointcloud(Path const &file);

 private:
  float *data_;
};

LidarScan LoadLidarScan(Path const folder, size_t const frame_id);

Eigen::Affine3d LoadLidarExtrinsics(kmc::Path const data_folder, bool const to_cam);

Image LoadImage(Path const folder, std::string const camera, size_t const frame_id);

Images LoadImages(Path const folder, size_t const frame_id);

Frame MakeFrame(kmc::Oxts const &odometry_n_m_1, kmc::Oxts const &odometry_n, kmc::Oxts const &odometry_n_p_1,
                kmc::LidarScan const &lidar_scan, std::optional<kmc::Images> const camera_images = std::nullopt);

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id, bool const load_images = false);

void WritePointcloud(Path const data_folder, size_t const frame_id, Pointcloud const &pointcloud,
                     VectorXd const &intensities);

Eigen::Affine3d LoadLidarExtrinsics(kmc::Path const data_folder, bool const to_cam = true);

}  // namespace kmc

namespace kmc::viz {

CameraCalibration CalibrationLinesToCalibration(std::vector<std::string> const calibration_lines);

CameraCalibrations LoadCameraCalibrations(kmc::Path const data_folder);

void MakeOutputImageFolders(Path const output_folder);

void SaveImagesOnTopOfEachother(Images const &top_imgs, Images const &bottom_imgs, size_t const frame_id,
                                Path const output_folder);

}  // namespace kmc::viz