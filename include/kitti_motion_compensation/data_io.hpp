#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Time LoadTimeStamp(Path const timestamp_file, size_t const frame_id);

Oxts LoadOxts(Path const folder, size_t const frame_id);

Pointcloud LoadPointcloud(Path const pointcloud_file);

LidarScan LoadLidarScan(Path const folder, size_t const frame_id);

Image LoadImage(Path const folder, std::string const camera,
                size_t const frame_id);

Images LoadImages(Path const folder, size_t const frame_id);

Frame LoadSingleFrame(Path const data_folder, size_t const frame_id,
                      bool const load_images = false);

} // namespace kmc

namespace kmc::viz {

CameraCalibration
CalibrationLinesToCalibration(std::vector<std::string> const calibration_lines);

CameraCalibrations LoadCameraCalibrations(kmc::Path const data_folder);

Eigen::Affine3d LoadLidarExtrinsics(kmc::Path const data_folder);

} // namespace kmc::viz