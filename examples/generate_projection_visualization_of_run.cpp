#include <string>
#include <vector>

#include "kitti_motion_compensation/data_io.hpp"
#include "kitti_motion_compensation/handlers.hpp"

using namespace kmc;

std::string const DATA_DIR{"../data/2011_09_26/"};
std::string const RUN{"2011_09_26_drive_0117_sync"};

std::string const OUTPUT_DIR{""};

int main() {
  viz::CameraCalibrations const camera_calibrations{viz::LoadCameraCalibrations(DATA_DIR)};
  Eigen::Affine3d const lidar_extrinsics{viz::LoadLidarExtrinsics(DATA_DIR)};

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
  GenerateProjectionVisualizationOfRun(camera_calibrations, lidar_extrinsics, Path{DATA_DIR + RUN}, Path{OUTPUT_DIR});

  return 0;
}