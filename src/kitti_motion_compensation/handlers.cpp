#include "kitti_motion_compensation/handlers.hpp"

#include <algorithm>
#include <filesystem>

#include "kitti_motion_compensation/camera_model.hpp"
#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"

namespace kmc {

namespace fs = std::filesystem;

std::size_t NumberOfFilesInDirectory(fs::path path) {
  return (std::size_t)std::distance(fs::directory_iterator{path}, fs::directory_iterator{});
}

void MotionCompensateRun(Path const run_folder) {
  Path const velodyne_folder{run_folder / Path{"velodyne_points"}};

  // we need to figure out how many frames there are in the run, so check the
  // length of the velodyne_points data folder. We could have checked any data
  // folder but this one was close by
  size_t number_of_frames{NumberOfFilesInDirectory(velodyne_folder / Path("data"))};

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
    Time const requested_time{frame.scan.stamp_middle};
    Pointcloud const motion_compensated_pointcloud{MotionCompensateFrame(frame, requested_time)};

    WritePointcloud(data_folder, i, motion_compensated_pointcloud, frame.scan.intensities);

    std::cout << "Motion compensated pointcloud number: " << i << std::endl;
  }
}

void GenerateProjectionVisualizationOfRun(viz::CameraCalibrations const camera_calibrations,
                                          Eigen::Affine3d const lidar_extrinsics, Path const run_folder,
                                          Path const output_folder) {
  // could use any folder to find out how many frames we have, here we just
  // happen to do it with the velodyne data folder
  Path const velodyne_folder{run_folder / Path{"velodyne_points"}};
  size_t number_of_frames{NumberOfFilesInDirectory(velodyne_folder / Path("data"))};

  viz::MakeOutputImageFolders(output_folder);

  for (size_t i{0}; i < number_of_frames; ++i) {
    // load the frame and project the raw pointcloud
    Frame frame{LoadSingleFrame(run_folder, i, true)};
    Images const projected_imgs_raw{viz::ProjectPointcloudOnFrame(frame, camera_calibrations, lidar_extrinsics)};

    // motion compensate the pointcloud and edit frame
    Time const requested_time{frame.scan.stamp_middle};
    Pointcloud const motion_compensated_pointcloud{MotionCompensateFrame(frame, requested_time)};

    frame.scan.cloud = motion_compensated_pointcloud;

    // project the motion compensated pointcloud
    Images const projected_imgs_mc{viz::ProjectPointcloudOnFrame(frame, camera_calibrations, lidar_extrinsics)};

    viz::SaveImagesOnTopOfEachother(projected_imgs_raw, projected_imgs_mc, i, output_folder);

    std::cout << "Projected and saved frame: " << i << std::endl;
  }
}

}  // namespace kmc
