#include "calibmar/readers/filesystem_reader.h"

#include <algorithm>
#include <filesystem>

namespace calibmar {

  FilesystemImageReader::FilesystemImageReader(const Options& options)
      : options_(options), file_index_(0), image_width_(-1), image_height_(-1) {
    std::replace(options_.image_directory.begin(), options_.image_directory.end(), '\\', '/');

    for (auto const& dir_entry : std::filesystem::directory_iterator{options_.image_directory}) {
      if (dir_entry.is_regular_file()) {
        image_paths_.push_back(dir_entry.path().string());
      }
    }
  }

  bool FilesystemImageReader::HasNext() {
    return file_index_ < image_paths_.size();
  }

  FilesystemImageReader::Status FilesystemImageReader::Next(Image& image, Pixmap& pixmap) {
    if (!HasNext()) {
      return Status::NO_MORE_IMAGES;
    }

    std::string path = image_paths_[file_index_];
    image.SetName(path);
    file_index_++;

    if (!pixmap.Read(path)) {
      return Status::READ_ERROR;
    }

    if (image_width_ == -1 && image_height_ == -1) {
      image_width_ = pixmap.Width();
      image_height_ = pixmap.Height();
    }
    else if (image_width_ != pixmap.Width() || image_height_ != pixmap.Height()) {
      return Status::IMAGE_DIMENSION_MISSMATCH;
    }

    return Status::SUCCESS;
  }

  int FilesystemImageReader::ImagesWidth() {
    return image_width_;
  }

  int FilesystemImageReader::ImagesHeight() {
    return image_height_;
  }
}