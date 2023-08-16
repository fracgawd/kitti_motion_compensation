#pragma once

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

std::size_t NumberOfFilesInDirectory(std::filesystem::path path);

void MotionCompensateRun(Path const run_folder);

void GenerateProjectionVisualizationOfRun(viz::CameraCalibrations const camera_calibrations,
                                          Eigen::Affine3d const lidar_extrinsics, Path const run_folder,
                                          Path const output_folder);

}  // namespace kmc
