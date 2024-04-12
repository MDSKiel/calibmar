#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace calibmar {
  // Pixmap is a pixel based image representation. Currently a wrapper around cv::Mat.
  class Pixmap {
   public:
    enum class ReadMode {
      GRAYSCALE = cv::ImreadModes::IMREAD_GRAYSCALE,
      COLOR = cv::ImreadModes::IMREAD_COLOR,
      COLOR_AS_SOURCE = cv::ImreadModes::IMREAD_ANYCOLOR
    };

    Pixmap();
    // Copy constructor, does not actually copy. Uses cv::Mat to act like a smart pointer.
    Pixmap(const Pixmap& other);
    // Move constructor
    Pixmap(Pixmap&& other);

    // Copy assignment, does not actually copy. Uses cv::Mat to act like a smart pointer.
    Pixmap& operator=(const Pixmap& other);
    // Move assignment.
    Pixmap& operator=(Pixmap&& other);

    Pixmap Clone();

    // Read from file
    bool Read(const std::string& path, const ReadMode mode = ReadMode::COLOR);
    // Write to file
    bool Write(const std::string& path);
    // Assign existing cv::Mat. Uses the cv::Mat(cv::Mat) constructor, that does not copy the data.
    // Returns false if the mat is not two dimensional, not 8 bit color or grayscale
    bool Assign(const cv::Mat& mat);

    cv::Mat& Data();
    const cv::Mat& Data() const;

    int Width();
    const int Width() const;
    int Height();
    const int Height() const;

   private:
    cv::Mat data_;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////

  inline cv::Mat& Pixmap::Data() {
    return data_;
  }

  inline const cv::Mat& Pixmap::Data() const {
    return data_;
  }

  inline int Pixmap::Width() {
    return data_.cols;
  }

  inline const int Pixmap::Width() const {
    return data_.cols;
  }

  inline int Pixmap::Height() {
    return data_.rows;
  }

  inline const int Pixmap::Height() const {
    return data_.rows;
  }
}
