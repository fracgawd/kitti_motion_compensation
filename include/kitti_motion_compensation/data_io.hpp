#pragma once

#include <Eigen/Dense>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

Time LoadTimeStamp(Path const timestamp_file, size_t const frame_id);

std::optional<Oxts> LoadOxts(Path const folder, size_t const frame_id);

// Implementation adopted from https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
Eigen::Affine3d OxtsToPose(Oxts const &odometry, double const scale = 1.0);

std::tuple<Pointcloud, VectorXd> LoadPointcloud(Path const pointcloud_file);

VectorXd GetPseudoTimeStamps(Pointcloud const &cloud, Time const start_time, Time const end_time);

LidarScan LoadLidarScan(Path const folder, size_t const frame_id);

Eigen::Affine3d LoadLidarExtrinsics(kmc::Path const data_folder, bool const to_cam);

Image LoadImage(Path const folder, std::string const camera, size_t const frame_id);

Images LoadImages(Path const folder, size_t const frame_id);

Affine3d InterpolatePose(Affine3d const &pose_0, Affine3d const &pose_1, double const x);

Affine3d InterpolateFramePose(kmc::Oxts const &odometry_0, kmc::Oxts const &odometry_1, kmc::Time requested_time);

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