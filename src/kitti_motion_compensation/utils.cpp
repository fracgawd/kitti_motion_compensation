#include "kitti_motion_compensation/utils.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>

namespace kmc {

std::string IdToZeroPaddedString(size_t const id, size_t const pad) {
  std::string const id_string{std::to_string(id)};
  // adopted from
  // https://stackoverflow.com/questions/6143824/add-leading-zeroes-to-string-without-sprintf
  return std::string(pad - std::min(pad, id_string.length()), '0') + id_string;
}

std::vector<std::string> TokenizeString(std::string raw_string) {
  // adopted from
  // https://stackoverflow.com/questions/13096719/read-input-numbers-separated-by-spaces
  std::istringstream raw_string_stream(raw_string);
  std::string token;

  std::vector<std::string> tokenized_string;
  while (std::getline(raw_string_stream, token, ' ')) {
    tokenized_string.push_back(token);
  }

  return tokenized_string;
}

double MmHhSsToSeconds(std::string const mm_hh_ss) {
  // times in the timestamp files look like: 13:04:34.309763177
  int const hours{std::stoi(mm_hh_ss.substr(0, 2))};
  int const minutes{std::stoi(mm_hh_ss.substr(3, 5))};
  double const seconds{std::stod(mm_hh_ss.substr(6, 18))};

  return static_cast<double>((60 * hours * 60) + (minutes * 60)) + seconds;
}

} // namespace kmc