#pragma once

#include <string>
#include <vector>

namespace kmc {

std::string IdToZeroPaddedString(size_t const id, size_t const pad = 10);

std::vector<std::string> TokenizeString(std::string string);

} // namespace kmc
