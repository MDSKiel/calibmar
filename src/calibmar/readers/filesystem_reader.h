#pragma once

#include "calibmar/readers/image_reader.h"
#include <shared_mutex>

namespace calibmar {
  // ImageReader implementation for reading images from the filesystem
  class FilesystemImageReader : public ImageReader {
   public:
    struct Options {
      std::string image_directory = "";
      Pixmap::ReadMode image_read_mode = Pixmap::ReadMode::COLOR;
      bool init_image_size = true;
    };

    FilesystemImageReader(const Options& options);

    bool HasNext();
    ImageReader::Status Next(Image& image, Pixmap& pixmap);
    int ImagesWidth();
    int ImagesHeight();

   private:
    // Image reader options.
    Options options_;
    // Index of previously processed image.
    size_t file_index_;
    // Images potentially to be read.
    std::vector<std::string> image_paths_;

    int image_width_;
    int image_height_;
    std::shared_mutex mutex_;
  };
}
