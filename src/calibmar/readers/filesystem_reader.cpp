
#include "calibmar/core/natural_sort.h"

#include "filesystem_reader.h"

#include <algorithm>
#include <filesystem>

namespace calibmar {

  FilesystemImageReader::FilesystemImageReader(const Options& options)
      : options_(options), file_index_(0), image_width_(-1), image_height_(-1) {
    std::replace(options_.image_directory.begin(), options_.image_directory.end(), '\\', '/');
    bool image_size_is_init = false;
    Pixmap pixmap;
    for (auto const& dir_entry : std::filesystem::directory_iterator{options_.image_directory}) {
      if (dir_entry.is_regular_file()) {
        if (options_.init_image_size && !image_size_is_init && pixmap.Read(dir_entry.path().string())) {
          image_height_ = pixmap.Height();
          image_width_ = pixmap.Width();
          image_size_is_init = true;
        }

        image_paths_.push_back(dir_entry.path().string());
      }
    }

    std::sort(image_paths_.begin(), image_paths_.end(), natural_sort::compare<std::string>);
  }

  bool FilesystemImageReader::HasNext() {
    // read lock
    std::shared_lock lock(mutex_);
    return file_index_ < image_paths_.size();
  }

  FilesystemImageReader::Status FilesystemImageReader::Next(Image& image, Pixmap& pixmap) {
    // write lock
    std::unique_lock lock(mutex_);
    if (file_index_ >= image_paths_.size()) {
      return Status::NO_MORE_IMAGES;
    }

    std::string path = image_paths_[file_index_];
    image.SetName(path);
    file_index_++;

    if (!pixmap.Read(path, options_.image_read_mode)) {
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
    if (options_.init_image_size && image_width_ == -1) {
      throw std::runtime_error("Could not find any image files! (Could not init image size!)");
    }

    return image_width_;
  }

  int FilesystemImageReader::ImagesHeight() {
    if (options_.init_image_size && image_height_ == -1) {
      throw std::runtime_error("Could not find any image files! (Could not init image size!)");
    }

    return image_height_;
  }
}