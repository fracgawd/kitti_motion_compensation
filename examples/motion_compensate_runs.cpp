#include "kitti_motion_compensation/handlers.hpp"

#include <string>
#include <vector>

using namespace kmc;

std::string const DATA_DIR{"../data/2011_09_26/"};

std::vector<std::string> const RUNS{
    "2011_09_26_drive_0001_sync", "2011_09_26_drive_0005_sync",
    "2011_09_26_drive_0091_sync", "2011_09_26_drive_0117_sync",
    "2011_09_26_drive_0104_sync"};

int main() {
  for (auto const &run : RUNS) {
    std::cout << "Motion compensating: " << run << std::endl;

    kmc::MotionCompensateRun(kmc::Path{DATA_DIR + run});
  }

  return 0;
}