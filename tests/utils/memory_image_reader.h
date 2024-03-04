#pragma once

#include "calibmar/readers/image_reader.h"

class MemoryImageReader : public calibmar::ImageReader {
 public:
  enum class Images { BasicChessboard };

  MemoryImageReader(const std::vector<Images>& images);

  bool HasNext();
  calibmar::ImageReader::Status Next(calibmar::Image& image, calibmar::Pixmap& pixmap);
  int ImagesWidth();
  int ImagesHeight();

 private:
  size_t file_index_;
  std::vector<calibmar::Pixmap> images_;
  int image_width_;
  int image_height_;
};