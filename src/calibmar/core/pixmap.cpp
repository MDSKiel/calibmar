#include "calibmar/core/pixmap.h"

#include <opencv2/imgcodecs.hpp>

namespace calibmar {

  Pixmap::Pixmap() {}

  Pixmap::Pixmap(const Pixmap& other) {
    data_ = cv::Mat(other.data_);
  }

  Pixmap::Pixmap(Pixmap&& other) {
    data_ = cv::Mat(other.data_);
  }

  Pixmap& Pixmap::operator=(const Pixmap& other) {
    data_ = cv::Mat(other.data_);
    return *this;
  }

  Pixmap& Pixmap::operator=(Pixmap&& other) {
    data_ = cv::Mat(other.data_);
    return *this;
  }

  Pixmap Pixmap::Clone() {
    Pixmap pixmap;
    pixmap.data_ = this->data_.clone();
    return pixmap;
  }

  bool calibmar::Pixmap::Assign(const cv::Mat& mat) {
    if (mat.dims > 2) {
      return false;
    }
    if (mat.depth() != CV_8U) {
      return false;
    }
    if (mat.channels() != 1 && mat.channels() != 3) {
      return false;
    }

    data_ = cv::Mat(mat);
    return true;
  }

  bool Pixmap::Read(const std::string& path, const ReadMode mode) {
    cv::Mat image = cv::imread(path, static_cast<int>(mode));

    if (image.data == NULL) {
      return false;
    }

    // Mat behaves as a smart pointer.
    data_ = image;

    return true;
  }
}