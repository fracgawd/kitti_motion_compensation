#include <string>
#include <vector>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/handlers.hpp"

using namespace kmc;

int main(int const argc, char const* const argv[]) {
  if (4 != argc) {
    std::cout
        << "\nYou didn't pass the required number of command line arguments. You need to pass \n\n\t <DATA_DIR> <RUN> "
           "<OUTPUT_DIR> \n\nFor example: ../assets/2011_09_26/ 2011_09_26_drive_0005_sync/ ../ \n"
        << std::endl;
    return 0;
  }

  std::string const data_dir{argv[1]};
  std::string const run{argv[2]};
  std::string const output_dir{argv[3]};

  viz::CameraCalibrations const camera_calibrations{viz::LoadCameraCalibrations(data_dir)};
  Eigen::Affine3d const lidar_extrinsics{LoadLidarExtrinsics(data_dir, true)};

  // When you ask yourself why this function is so slow - remember what it does:
  //
  // 1) load the pointcloud and 4 images
  // 2) project the pointloud onto the four images individualy and save them
  // 3) motion compensate the pointcloud
  // 4) project the pointcloud into the four images individualy and save them
  // again!!!
  //
  // That is a lot of work to do if you ask me. On my laptop to process a single
  // frame it takes about 2 seconds
  GenerateProjectionVisualizationOfRun(camera_calibrations, lidar_extrinsics, Path{data_dir + run}, Path{output_dir});

  return 0;
}