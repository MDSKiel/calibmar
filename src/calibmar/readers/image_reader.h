#pragma once

#include "calibmar/core/image.h"
#include "calibmar/core/pixmap.h"

#include <vector>

namespace calibmar {
  // Abstract class for image readers
  class ImageReader {
   public:
    enum class Status { READ_ERROR, IMAGE_DIMENSION_MISSMATCH, NO_MORE_IMAGES, SUCCESS };

    // If there are still files to read
    virtual bool HasNext() = 0;
    // Read the next file. Set image width and height if not yet set.
    virtual Status Next(Image& image, Pixmap& pixmap) = 0;
    // Width of the first read image or -1 if no image has been read.
    virtual int ImagesWidth() = 0;
    // Height of the first read image or -1 if no image has been read.
    virtual int ImagesHeight() = 0;
    virtual ~ImageReader() = default;
  };
}