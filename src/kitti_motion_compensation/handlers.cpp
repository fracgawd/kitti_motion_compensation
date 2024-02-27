#include "kitti_motion_compensation/handlers.hpp"

#include <algorithm>
#include <filesystem>

#include "kitti_motion_compensation/camera_model.hpp"
#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/motion_compensation.hpp"
#include "kitti_motion_compensation/utils.hpp"

namespace kmc {

namespace fs = std::filesystem;

std::size_t NumberOfFilesInDirectory(fs::path path) {
  return (std::size_t)std::distance(fs::directory_iterator{path}, fs::directory_iterator{});
}

void CopyOverUncompensatedFirstAndLastFrame(Path const run_folder) {
  // TODO(jack): we are missing an abstraction here! This code is copying and pasting and just in general gets the job
  // done but doesnt do it nicely!
  Path const velodyne_folder{run_folder / Path{"velodyne_points"}};
  KittiPclLoader pcl_loader_handler;

  // first frame
  size_t const first_frame_id{0};
  Path const first_pointcloud_file(velodyne_folder / Path("data/" + IdToZeroPaddedString(first_frame_id) + ".bin"));
  auto const [first_pointcloud, first_intensities] = pcl_loader_handler.LoadPointcloud(first_pointcloud_file);
  WritePointcloud(velodyne_folder / Path("data_motion_compensated/"), first_frame_id, first_pointcloud,
                  first_intensities);

  // last frame
  size_t const number_of_frames{NumberOfFilesInDirectory(velodyne_folder / Path("data"))};
  size_t const last_frame_id{number_of_frames - 1};
  Path const last_pointcloud_file(velodyne_folder / Path("data/" + IdToZeroPaddedString(last_frame_id) + ".bin"));
  auto const [last_pointcloud, last_intensities] = pcl_loader_handler.LoadPointcloud(first_pointcloud_file);
  WritePointcloud(velodyne_folder / Path("data_motion_compensated/"), last_frame_id, first_pointcloud,
                  first_intensities);
}

void MotionCompensateRun(Path const run_folder) {
  Path const velodyne_folder{run_folder / Path{"velodyne_points"}};
  size_t const number_of_frames{NumberOfFilesInDirectory(velodyne_folder / Path("data"))};

  Path const data_folder{velodyne_folder / Path("data_motion_compensated")};
  if (not fs::is_directory(data_folder) || not fs::exists(data_folder)) {
    fs::create_directory(data_folder);
  }

  // The first and last frames cannot be motion compensated because there is not odometry data from the before AND after
  // frame which is needed to interpolate the start and end pose of the frame
  CopyOverUncompensatedFirstAndLastFrame(run_folder);

  // TODO(jack): handle loading the sequence and the first and last frame
  for (size_t i{1}; i < number_of_frames - 1; ++i) {
    Frame const frame{LoadSingleFrame(run_folder, i)};

    // The middle of the scan which is when the lidar triggers the cameras
    Time const requested_time{frame.scan.stamp_middle};
    Pointcloud const motion_compensated_pointcloud{MotionCompensateFrame(frame, requested_time)};

    WritePointcloud(data_folder, i, motion_compensated_pointcloud, frame.scan.intensities);
    std::cout << "Motion compensated pointcloud number: " << i << std::endl;
  }
}

void GenerateProjectionVisualizationOfRun(viz::CameraCalibrations const camera_calibrations,
                                          Eigen::Affine3d const lidar_extrinsics, Path const run_folder,
                                          Path const output_folder) {
  Path const velodyne_folder{run_folder / Path{"velodyne_points"}};
  size_t const number_of_frames{NumberOfFilesInDirectory(velodyne_folder / Path("data"))};

  viz::MakeOutputImageFolders(output_folder);

  // TODO(jack): handle loading the sequence and the first and last frame
  for (size_t i{1}; i < number_of_frames - 1; ++i) {
    // load the frame and project the raw pointcloud
    Frame frame{LoadSingleFrame(run_folder, i, true)};
    Images const projected_imgs_raw{viz::ProjectPointcloudOnFrame(frame, camera_calibrations, lidar_extrinsics)};

    // motion compensate
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
