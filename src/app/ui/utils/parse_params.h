#pragma once

#include <colmap/util/misc.h>
#include <vector>

namespace calibmar {
  //inline bool TryParseParams(std::vector<double>& params, const std::string& params_string);

  inline bool TryParseParams(std::vector<double>& params, const std::string& params_string) {
    std::vector<double> parsed = colmap::CSVToVector<double>(params_string);
    if (parsed.size() == 0) {
      return false;
    }
    for (auto d : parsed) {
      params.push_back(d);
    }
    return true;
  }
}