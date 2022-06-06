#include "livestream_reader.h"

namespace calibmar {

  LiveStreamImageReader::LiveStreamImageReader(const Options& options)
      : options_(options), has_next_(true), image_width_(-1), image_height_(-1) {}

  bool LiveStreamImageReader::HasNext() {
    return has_next_;
  }

  ImageReader::Status LiveStreamImageReader::Next(Image& image, Pixmap& pixmap) {
    if (!has_next_) {
      return ImageReader::Status::NO_MORE_IMAGES;
    }
    if (!Open()) {
      has_next_ = false;
      return ImageReader::Status::NO_MORE_IMAGES;
    }

    if (!video_capture_.read(pixmap.Data())) {
      has_next_ = false;
      return ImageReader::Status::NO_MORE_IMAGES;
    }

    if (image_width_ != pixmap.Width() || image_height_ != pixmap.Height()) {
      return Status::IMAGE_DIMENSION_MISSMATCH;
    }

    return ImageReader::Status::SUCCESS;
  }

  bool LiveStreamImageReader::Open() {
    if (video_capture_.isOpened()) {
      return true;
    }

    if (video_capture_.open(options_.device_id, cv::CAP_ANY)) {
      image_width_ = video_capture_.get(cv::CAP_PROP_FRAME_WIDTH);
      image_height_ = video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
      return true;
    }
    else {
      return false;
    }
  }

  void LiveStreamImageReader::Close() {
    video_capture_.release();
    has_next_ = false;
  }

  int LiveStreamImageReader::ImagesWidth() {
    return image_width_;
  }

  int LiveStreamImageReader::ImagesHeight() {
    return image_height_;
  }
}