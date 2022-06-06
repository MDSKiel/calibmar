#include "memory_image_reader.h"
#include "test_images.h"

namespace {

  std::vector<uchar>& GetImageData(MemoryImageReader::Images image) {
    switch (image)
    {
    case MemoryImageReader::Images::BasicChessboard:
      return test_images::chessboard_image;
    default:
      throw std::runtime_error("bad image");
    }
  }
}


MemoryImageReader::MemoryImageReader(const std::vector<Images>& images) : file_index_(0), image_width_(-1), image_height_(-1) {
  for (Images image : images) {
    calibmar::Pixmap img;
    std::vector<uchar>& img_data = GetImageData(image);
      cv::Mat raw(1, img_data.size(), CV_8U, img_data.data());
    img.Assign(cv::imdecode(raw, cv::ImreadModes::IMREAD_GRAYSCALE));
    images_.push_back(img);
  }
}

bool MemoryImageReader::HasNext() {
  return file_index_ < images_.size();
}

calibmar::ImageReader::Status MemoryImageReader::Next(calibmar::Image& image, calibmar::Pixmap& pixmap) {
  if (!HasNext()) {
    return Status::NO_MORE_IMAGES;
  }

  cv::Mat& mat = images_[file_index_].Data();
  file_index_++;

  if (mat.dims != 2) {
    return Status::READ_ERROR;
  }

  if (image_width_ == -1 && image_height_ == -1) {
    image_width_ = mat.cols;
    image_height_ = mat.rows;
  } else if (image_width_ != mat.cols || image_height_ != mat.rows) {
    return Status::IMAGE_DIMENSION_MISSMATCH;
  }

  pixmap.Assign(mat);

  return ImageReader::Status::SUCCESS;
}

int MemoryImageReader::ImagesWidth() {
  return image_width_;
}

int MemoryImageReader::ImagesHeight() {
  return image_height_;
}



