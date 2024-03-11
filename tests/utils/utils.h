#pragma once

#include "calibmar/core/image.h"
#include "calibmar/core/pixmap.h"
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <iostream>

void DrawTargetOnImage(calibmar::Pixmap& image, const calibmar::Image& image_data) {
  if (image_data.ArucoKeypoints().size() > 0) {
    for (const auto& id_corners : image_data.ArucoKeypoints()) {
      int id = id_corners.first;
      const std::vector<Eigen::Vector2d>& corners = id_corners.second;

      cv::Point tl(corners[0].x(), corners[0].y());
      cv::Point tr(corners[1].x(), corners[1].y());
      cv::Point br(corners[2].x(), corners[2].y());
      cv::Point bl(corners[3].x(), corners[3].y());

      cv::line(image.Data(), tl, tr, cv::Scalar(0, 0, 255));
      cv::line(image.Data(), tr, br, cv::Scalar(0, 0, 255));
      cv::line(image.Data(), br, bl, cv::Scalar(0, 0, 255));
      cv::line(image.Data(), bl, tl, cv::Scalar(0, 0, 255));

      cv::rectangle(image.Data(), tl, tl, cv::Scalar(255, 0, 0));
      cv::rectangle(image.Data(), tr, tr, cv::Scalar(0, 255, 0));
      cv::rectangle(image.Data(), br, br, cv::Scalar(0, 255, 0));
      cv::rectangle(image.Data(), bl, bl, cv::Scalar(0, 255, 0));

      cv::putText(image.Data(), std::to_string(id), tl, cv::HersheyFonts::FONT_HERSHEY_DUPLEX, 0.25, cv::Scalar(0, 0, 255));
    }
  }
}
void PrintFileHex(const std::string& file_path) {
  const std::string inputFile = file_path;
  std::ifstream infile(inputFile, std::ios_base::binary);

  std::ios::fmtflags f(std::cout.flags());  // store cout format

  std::cout << "\nstd::vector<unsigned char> hex_values {" << std::endl;

  auto iter = std::istreambuf_iterator<char>(infile);
  auto end = std::istreambuf_iterator<char>{};

  std::cout << "0x" << std::hex << std::uppercase << (int)(static_cast<unsigned char>(*iter));
  iter++;
  for (; iter != end; iter++) {
    std::cout << ", 0x" << std::hex << std::uppercase << (int)(static_cast<unsigned char>(*iter));
  }

  std::cout << "};" << std::endl;

  std::cout.flags(f);  // restore cout format
}
