#include <string>
#include <vector>

#include "kitti_motion_compensation/handlers.hpp"

using namespace kmc;
namespace fs = std::filesystem;

int main(int const argc, char const* const argv[]) {
  std::vector<std::string> runs;
  if (2 == argc) {
    std::cout << "Motion compensating all runs found in data directory: " << argv[1] << std::endl;
    for (const auto& entry : fs::directory_iterator(argv[1])) {
      if (entry.is_directory()) {
        std::string const run_i{entry.path().filename()};
        std::cout << "\t" << run_i << std::endl;
        runs.push_back(run_i);
      }
    }
    std::cout << "\n" << std::endl;
  } else if (2 < argc) {
    std::cout << "Motion compensating the following runs in data directory: " << argv[1] << std::endl;
    for (int i = 2; i < argc; i++) {
      std::string const run_i{argv[i]};
      std::cout << "\t" << run_i << std::endl;
      runs.push_back(run_i);
    }
    std::cout << "\n" << std::endl;
  } else {
    std::cout << "You didn't pass the required command line arguments. Please pass the following command line "
                 "arguments:\n\n\t To motion compensate all runs in a provided data "
                 "directory:\n\t\t./motion_compensate_runs <DATA_DIR>\n\n\tTo motion compensate specific "
                 "runs:\n\t\t./motion_compensate_runs <DATA_DIR> <RUN_1> <RUN_2> <RUN_N>\n"
              << std::endl;
    return -1;
  }

  std::string const data_dir{argv[1]};

  for (auto const& run : runs) {
    std::cout << "Motion compensating: " << run << std::endl;

    kmc::MotionCompensateRun(kmc::Path{data_dir + run});
  }

  return 0;
}