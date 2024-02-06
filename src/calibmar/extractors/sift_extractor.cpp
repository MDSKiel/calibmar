#include "sift_extractor.h"
#include <colmap/feature/sift.h>
#include <opencv2/imgproc.hpp>

#include <FreeImage.h>

namespace calibmar {

  SiftFeatureExtractor::SiftFeatureExtractor(const Options& options) : options_(options) {
    colmap::SiftExtractionOptions sift_options;
#if defined(COLMAP_CUDA_ENABLED)
    sift_options.use_gpu = true;
#else
    sift_options.use_gpu = false;
#endif
    sift_extractor_ = colmap::CreateSiftFeatureExtractor(sift_options);
  }

  FeatureExtractor::Status SiftFeatureExtractor::Extract(Image& image, const Pixmap& pixmap) {
    cv::Mat image_data(pixmap.Data());

    if (pixmap.Data().channels() == 3) {
      image_data.create(pixmap.Data().rows, pixmap.Data().cols, CV_8U);
      cv::cvtColor(pixmap.Data(), image_data, cv::COLOR_BGR2GRAY);
    }

    FIBITMAP* bitmap = FreeImage_ConvertFromRawBitsEx(false, image_data.data, FREE_IMAGE_TYPE::FIT_BITMAP, pixmap.Width(),
                                                      pixmap.Height(), image_data.step, 8, 0, 0, 0, true);
                                                      
    colmap::Bitmap bmp(bitmap);
    colmap::FeatureKeypoints keypoints;
    colmap::FeatureDescriptors descriptors;
    if (!sift_extractor_->Extract(bmp, &keypoints, &descriptors)) {
      return FeatureExtractor::Status::DETECTION_ERROR;
    }

    std::vector<Eigen::Vector2d> points2D;
    points2D.reserve(keypoints.size());
    for (const auto& keypoint : keypoints) {
      points2D.push_back(Eigen::Vector2d(keypoint.x, keypoint.y));
    }
    image.SetPoints2D(points2D);
    image.SetDescriptors(descriptors);
    image.SetKeypoints(keypoints);

    return FeatureExtractor::Status::SUCCESS;
  }
}