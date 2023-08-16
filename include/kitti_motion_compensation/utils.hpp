#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "kitti_motion_compensation/data_types.hpp"

namespace kmc {

std::string IdToZeroPaddedString(size_t const id, size_t const pad = 10);

std::vector<std::string> TokenizeString(std::string string);

double MmHhSsToSeconds(std::string const mm_hh_ss);

}  // namespace kmc

namespace kmc::viz::utils {

void StackAndProcessProjectionImagePair(cv::Mat const &img_top, cv::Mat const &img_bottom, Path const output_file);
}  // namespace kmc::viz::utils
