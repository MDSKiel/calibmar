#pragma once

#include "calibmar/readers/image_reader.h"

#include <opencv2/videoio.hpp>

namespace calibmar {
  // ImageReader implementation for reading images from a camera device
  class LiveStreamImageReader : public ImageReader {
   public:
    struct Options {
      int device_id = 0;
    };

    LiveStreamImageReader(const Options& options);

    bool HasNext();
    ImageReader::Status Next(Image& image, Pixmap& pixmap);
    bool Open();
    void Close();
    int ImagesWidth();
    int ImagesHeight();

   private:
    Options options_;
    cv::VideoCapture video_capture_;
    int image_width_;
    int image_height_;
    bool has_next_;
  };
}