#include "calibmar/calibrators/opencv_calibration.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "calibmar/pose_suggestion/pose_suggestion.h"
#include "opencv2/core/utils/logger.hpp"
#include <Eigen/SparseCore>
#include <calibmar/calibrators/basic_calibrator.h>
#include <calibmar/calibrators/calibrator.h>
#include <calibmar/readers/filesystem_reader.h>
#include <colmap/geometry/pose.h>
#include <colmap/scene/projection.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace calibmar;

// This file is a loose list of functions and a command line interface that was used to manually verify
// implemented routines. Especially the pose suggestion procedure. Can be used as a basis for future work,
// but is generally not runnable as is.

namespace {

  // This is a custom transform to transform pose suggested poses (in camera coordinates) to
  // blender geodt world coordinates relative to the camera position and orientation there.
  // The board origin and orientation has been adapted inside geodt though.
  void TransformAndPrint(const Eigen::Vector4d& rot, const Eigen::Vector3d& trans, std::ostream& out) {
    Eigen::Translation3d camB_translate(1.15, 0, 0);
    Eigen::Affine3d camB_rotate(Eigen::AngleAxis(M_PI / 2, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxis(0.0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxis(M_PI / 2, Eigen::Vector3d::UnitX()));
    Eigen::Matrix4d camB = (camB_translate * camB_rotate).matrix();
    Eigen::Matrix4d camA = Eigen::Matrix4d::Identity();
    camA(1, 1) = -1;
    camA(2, 2) = -1;

    Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
    pose_mat.block<3, 3>(0, 0) = Eigen::Quaterniond(rot(0), rot(1), rot(2), rot(3)).matrix();
    pose_mat.block<3, 1>(0, 3) = trans;

    Eigen::Matrix4d pose_mat_trans = camB * camA * pose_mat * camA;
    Eigen::Matrix3d rot_trans = pose_mat_trans.block<3, 3>(0, 0);
    Eigen::Vector3d translation_trans = pose_mat_trans.block<3, 1>(0, 3);
    Eigen::Quaterniond quat(rot_trans);
    Eigen::Vector4d quat_trans(quat.w(), quat.x(), quat.y(), quat.z());

    Eigen::IOFormat CleanFmt(-1, 0, ", ", "\n", "[", "]");

    Eigen::Matrix<double, 7, 1> stack;
    stack << quat_trans, translation_trans;

    out << stack.transpose().format(CleanFmt);
  }

  // Used to replace a line in the config.yaml of geodt, to automatically render a new pose
  void ReplaceLine(const std::string& path, const std::string& value, int line) {
    std::string tmp_path = (std::filesystem::temp_directory_path() / std::filesystem::path("tmp.txt")).string();

    std::ofstream tmp_file;
    tmp_file.open(tmp_path);
    std::ifstream file(path);

    std::string line_str;
    int line_num = 0;
    while (std::getline(file, line_str)) {
      if (line_num == line) {
        tmp_file << value << std::endl;
      }
      else {
        tmp_file << line_str << std::endl;
      }
      line_num++;
    }
    file.close();
    tmp_file.close();

    std::filesystem::rename(tmp_path, path);
  }

  //   // exec and wait for a command line process on windows, used to call blender command line
  //   void exec(const char* cmd) {
  //     STARTUPINFO si;
  //     PROCESS_INFORMATION pi;

  //     ZeroMemory(&si, sizeof(si));
  //     si.cb = sizeof(si);
  //     ZeroMemory(&pi, sizeof(pi));

  //     LPSTR lpstr = const_cast<LPSTR>(cmd);

  //     // Start the child process.
  //     if (!CreateProcess(NULL,   // No module name (use command line)
  //                        lpstr,  // Command line
  //                        NULL,   // Process handle not inheritable
  //                        NULL,   // Thread handle not inheritable
  //                        FALSE,  // Set handle inheritance to FALSE
  //                        0,      // No creation flags
  //                        NULL,   // Use parent's environment block
  //                        NULL,   // Use parent's starting directory
  //                        &si,    // Pointer to STARTUPINFO structure
  //                        &pi)    // Pointer to PROCESS_INFORMATION structure
  //     ) {
  //       throw std::runtime_error("CreateProcess failed");
  //     }

  //     // Wait until child process exits.
  //     WaitForSingleObject(pi.hProcess, INFINITE);

  //     // Close process and thread handles.
  //     CloseHandle(pi.hProcess);
  //     CloseHandle(pi.hThread);
  //   }
  // }

  void DistortImages(const std::string& input, const std::string& output, cv::Mat& map_x, cv::Mat& map_y) {
    std::filesystem::path target_dir(output);

    for (auto const& dir_entry : std::filesystem::directory_iterator{input}) {
      cv::Mat input_mat = cv::imread(dir_entry.path().string());
      cv::Mat output_mat;
      cv::remap(input_mat, output_mat, map_x, map_y, cv::INTER_LINEAR);

      std::filesystem::path target_file = target_dir / dir_entry.path().filename();
      std::cout << target_file << std::endl;
      cv::imwrite(target_file.string(), output_mat);
    }
  }

  void CreateDistortMaps(cv::Mat* map_x, cv::Mat* map_y, const colmap::Camera& camera) {
    colmap::Camera undistorted_camera;
    undistorted_camera.SetModelId(colmap::PinholeCameraModel::model_id);
    undistorted_camera.SetWidth(camera.Width());
    undistorted_camera.SetHeight(camera.Height());
    // Copy focal length parameters.
    const std::vector<size_t>& focal_length_idxs = camera.FocalLengthIdxs();
    if (focal_length_idxs.size() == 1) {
      undistorted_camera.SetFocalLengthX(camera.FocalLength());
      undistorted_camera.SetFocalLengthY(camera.FocalLength());
    }
    else if (focal_length_idxs.size() == 2) {
      undistorted_camera.SetFocalLengthX(camera.FocalLengthX());
      undistorted_camera.SetFocalLengthY(camera.FocalLengthY());
    }
    // Copy principal point parameters.
    undistorted_camera.SetPrincipalPointX(camera.PrincipalPointX());
    undistorted_camera.SetPrincipalPointY(camera.PrincipalPointY());

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double max_y = std::numeric_limits<double>::min();
    cv::Size2i image_size(camera.Width(), camera.Height());
    *map_x = cv::Mat(image_size, CV_32FC1);
    *map_y = cv::Mat(image_size, CV_32FC1);
    for (int y = 0; y < image_size.height; ++y) {
      float* ptr1 = map_x->ptr<float>(y);
      float* ptr2 = map_y->ptr<float>(y);
      for (int x = 0; x < image_size.width; ++x) {
        Eigen::Vector2d undist_world = camera.CamFromImg({x, y});
        Eigen::Vector2d dist_img = undistorted_camera.ImgFromCam(undist_world);
        ptr1[x] = dist_img.x();
        ptr2[x] = dist_img.y();

        if (x == 0 && dist_img.x() < min_x) {
          min_x = dist_img.x();
        }
        else if (x == image_size.width - 1 && dist_img.x() > max_x) {
          max_x = dist_img.x();
        }
        if (y == 0 && dist_img.y() < min_y) {
          min_y = dist_img.y();
        }
        else if (y == image_size.height - 1 && dist_img.y() > max_y) {
          max_y = dist_img.y();
        }
      }
    }

    // if (min_x < 0 || min_y < 0) {
    //   cv::Rect crop(0 - min_x / 2, 0 - min_y / 2, image_size.width + min_x, image_size.height + min_y);
    //   std::cout << "Resulting image smaller, resizing: " << crop << std::endl;
    //   *map_x = (*map_x)(crop);
    //   *map_y = (*map_y)(crop);
    // }
  }

  // used to calculate the evolution of rms, std_dev and params for a set of images.
  // That is: calibrating for 1, 2, 3 ... n images and saving the results.
  void CalculateCalibrationSeries(const std::string& image_dir, std::ostream& out) {
    std::filesystem::path target_dir(image_dir);

    Calibration calibration;
    ChessboardFeatureExtractor::Options ex_options;
    ex_options.chessboard_columns = 7;
    ex_options.chessboard_rows = 8;
    ex_options.square_size = 0.04;
    ChessboardFeatureExtractor extractor(ex_options);
    calibration.SetPoints3D(extractor.Points3D());
    BasicCalibrator::Options cal_options;
    cal_options.camera_model = CameraModelType::RadialCameraModel;
    cal_options.image_size = {1280, 1024};
    BasicCalibrator calibrator(cal_options);

    std::vector<double> rms_errors;
    std::vector<std::vector<double>> std_devs;
    std::vector<std::vector<double>> params;

    Pixmap pixmap;
    Image image;
    for (auto const& dir_entry : std::filesystem::directory_iterator{target_dir}) {
      if (!dir_entry.is_regular_file()) {
        continue;
      }
      if (!pixmap.Read(dir_entry.path().string())) {
        continue;
      }
      if (extractor.Extract(image, pixmap) != FeatureExtractor::Status::SUCCESS) {
        continue;
      }
      calibration.AddImage(image);

      std::cout << dir_entry << std::endl;

      calibrator.Calibrate(calibration);

      rms_errors.push_back(calibration.CalibrationRms());
      std_devs.push_back(calibration.IntrinsicsStdDeviations());
      params.push_back(calibration.Camera().Params());
    }

    out << "{" << std::endl;
    out << "\"RMSs " << rms_errors.size() << "\":" << std::endl;
    out << "[ " << rms_errors[0];
    for (size_t i = 1; i < rms_errors.size(); i++) {
      out << ", " << rms_errors[i];
    }
    out << "]," << std::endl << std::endl;

    out << "\"STDs " << std_devs.size() << "\":" << std::endl << "[" << std::endl;
    for (size_t i = 0; i < std_devs[0].size(); i++) {
      out << "[" << std_devs[0][i];
      for (size_t j = 1; j < std_devs.size(); j++) {
        out << ", " << std_devs[j][i];
      }
      out << "]," << std::endl;
    }
    out << "]," << std::endl << std::endl;

    out << "\"Params " << params.size() << "\":" << std::endl << "[" << std::endl;
    for (size_t i = 0; i < params[0].size(); i++) {
      out << "[" << params[0][i];
      for (size_t j = 1; j < params.size(); j++) {
        out << ", " << params[j][i];
      }
      out << "]," << std::endl;
    }
    out << "]" << std::endl << "}" << std::endl;
  }

  // Generate blender rendered images from the calibmar::pose_suggestion poses.
  // Uses a modified version (board origin and initial orientation changed to match pose_suggest camera coordinates)
  // of the GEOMAR geodt blender project.
  // Will continue with the current images in the target_dir.
  void GeneratePoseSuggestionImages(const std::string& blender_img_output_dir, const std::string& target_dir,
                                    const std::string& blender_cfg_file, int line_to_replace,
                                    const std::string& blender_cmd_line_call, colmap::Camera& camera,
                                    std::optional<std::tuple<cv::Mat&, cv::Mat&>> maps = {}) {
    Calibration calibration;
    ChessboardFeatureExtractor::Options ex_options;
    ex_options.chessboard_columns = 7;
    ex_options.chessboard_rows = 8;
    ex_options.square_size = 0.04;
    ChessboardFeatureExtractor extractor(ex_options);
    calibration.SetPoints3D(extractor.Points3D());
    BasicCalibrator::Options cal_options;
    cal_options.camera_model = CameraModel::IdToCameraModelType().at(camera.ModelId());
    cal_options.image_size = maps.has_value()
                                 ? std::pair<int, int>{std::get<0>(maps.value()).cols, std::get<0>(maps.value()).rows}
                                 : std::pair<int, int>{camera.Width(), camera.Height()};
    BasicCalibrator calibrator(cal_options);

    std::filesystem::path target_directory(target_dir);
    for (int i = 0; i < 100 && calibration.Images().size() < 30; i++) {
      char file_name[] = "0000";
      std::sprintf(file_name, "%.4i", i);
      std::filesystem::path current_file(target_directory / std::filesystem::path(std::string(file_name) + ".png"));
      Pixmap pixmap;
      Image image;

      if (std::filesystem::exists(current_file)) {
        pixmap.Read(current_file.string());
        if (extractor.Extract(image, pixmap) != FeatureExtractor::Status::SUCCESS) {
          continue;
        }
        calibration.AddImage(image);
        continue;
      }

      if (calibration.Images().size() > 0) {
        calibrator.Calibrate(calibration);
      }

      Eigen::Vector3d trans;
      Eigen::Vector4d rot;
      pose_suggestion::EstimateNextBestPose(calibration, 6, 7, 0.04, rot, trans);

      std::stringstream ss;
      TransformAndPrint(rot, trans, ss);
      ReplaceLine(blender_cfg_file, ss.str(), line_to_replace);
      // exec(blender_cmd_line_call.c_str());

      std::filesystem::rename(std::filesystem::path(blender_img_output_dir) / "0000.png", current_file);

      if (maps.has_value()) {
        cv::Mat& map_x = std::get<0>(maps.value());
        cv::Mat& map_y = std::get<1>(maps.value());
        cv::Mat in, out;
        in = cv::imread(current_file.string());
        cv::remap(in, out, map_x, map_y, cv::InterpolationFlags::INTER_LINEAR);
        cv::imwrite(current_file.string(), out);
      }

      pixmap.Read(current_file.string());

      cv::Mat noise(pixmap.Data().size(), pixmap.Data().type());
      cv::randn(noise, 0, 10);  // add noise
      pixmap.Data() += noise;
      cv::imwrite(current_file.string(), pixmap.Data());

      if (extractor.Extract(image, pixmap) != FeatureExtractor::Status::SUCCESS) {
        std::filesystem::path error_name = current_file;
        // rename if not extractable
        std::filesystem::rename(current_file, error_name.replace_extension("x" + current_file.extension().string()));
        continue;
      }
      calibration.AddImage(image);
    }

    std::cout << "calc series" << std::endl;

    std::ofstream out((target_directory / "calibration_series.txt"));
    CalculateCalibrationSeries(target_directory.string(), out);
    out.close();
  }

  void ProcessImages(const std::string& source_dir, const std::string& target_dir, int count, std::tuple<cv::Mat&, cv::Mat&> maps,
                     int sigma) {
    ChessboardFeatureExtractor::Options ex_options;
    ex_options.chessboard_columns = 7;
    ex_options.chessboard_rows = 8;
    ex_options.square_size = 0.04;
    ChessboardFeatureExtractor extractor(ex_options);

    Image image;
    Pixmap pixmap;
    int extracted = 0;
    for (auto const& dir_entry : std::filesystem::directory_iterator{source_dir}) {
      if (!dir_entry.is_regular_file() || dir_entry.path().extension() != ".png") {
        continue;
      }
      std::cout << dir_entry << std::endl;

      std::filesystem::path target_file(target_dir / dir_entry.path().filename());
      cv::Mat& map_x = std::get<0>(maps);
      cv::Mat& map_y = std::get<1>(maps);
      cv::Mat in, out;
      in = cv::imread(dir_entry.path().string());
      cv::remap(in, out, map_x, map_y, cv::InterpolationFlags::INTER_LINEAR);

      if (sigma > 0) {  // add noise
        cv::Mat noise(out.size(), out.type());
        cv::randn(noise, 0, sigma);
        out += noise;
      }
      cv::imwrite(target_file.string(), out);

      pixmap.Assign(out);
      if (extractor.Extract(image, pixmap) != FeatureExtractor::Status::SUCCESS) {
        if (target_file.filename().string().find('x') == std::string::npos) {
          std::filesystem::path error_name = target_file;
          // rename if not extractable (and not already renamed)
          std::filesystem::rename(target_file, error_name.replace_extension("x" + target_file.extension().string()));
        }
      }
      else {
        extracted++;
      }

      if (extracted >= count) {
        break;
      }
    }

    std::ofstream out(std::filesystem::path(target_dir) / "calibration_series.txt");
    CalculateCalibrationSeries(target_dir, out);
    out.close();
  }
}

int main(int argc, char* argv[]) {
  if (argv[1] == std::string("0")) {
    std::cout << "Calculate Calibration Series" << std::endl;

    if (argc == 3) {
      CalculateCalibrationSeries(argv[2], std::cout);
    }
    else if (argc == 4) {
      std::ofstream out(argv[3]);
      CalculateCalibrationSeries(argv[2], out);
      out.close();
    }
  }
  else if (argv[1] == std::string("1")) {
    std::cout << "Eval Pose Suggestion" << std::endl;

    auto camera = CameraModel::InitCamera(CameraModelType::RadialCameraModel, {1280, 1024}, {1108.5, 640, 512, 0.5, 1});
    cv::Mat map_x, map_y;
    CreateDistortMaps(&map_x, &map_y, camera);
    std::tuple<cv::Mat&, cv::Mat&> maps(map_x, map_y);

    GeneratePoseSuggestionImages(
        R"(E:\Code\geodt\pose_suggest\No_Water_Dome_Port_GEOMAR_A3)",
        R"(E:\Code\geodt\pose_suggest\No_Water_Dome_Port_GEOMAR_A3_output)", R"(E:\Code\geodt\config\pose_suggest.yaml)", 25,
        R"("C:\Program Files\Blender Foundation\Blender 3.0\blender.exe" "E:\Code\geodt\geodt2.blend" --background --python-text render_dataset)",
        camera, maps);
  }
  else if (argv[1] == std::string("2")) {
    std::cout << "Check and rename if not extractable" << std::endl;

    ChessboardFeatureExtractor::Options ex_options;
    ex_options.chessboard_columns = 7;
    ex_options.chessboard_rows = 8;
    ex_options.square_size = 0.04;
    ChessboardFeatureExtractor extractor(ex_options);

    Image image;
    Pixmap pixmap;
    int extracted = 0;
    for (auto const& dir_entry : std::filesystem::directory_iterator{argv[2]}) {
      if (!dir_entry.is_regular_file() || dir_entry.path().extension() != ".png") {
        continue;
      }

      std::cout << dir_entry << std::endl;
      pixmap.Read(dir_entry.path().string());
      if (extractor.Extract(image, pixmap) != FeatureExtractor::Status::SUCCESS) {
        if (dir_entry.path().filename().string().find('x') == std::string::npos) {
          std::filesystem::path error_name = dir_entry.path();
          // rename if not extractable (and not already renamed)
          std::filesystem::rename(dir_entry.path(), error_name.replace_extension("x" + dir_entry.path().extension().string()));
        }
      }
      else {
        extracted++;
      }
    }

    std::cout << "extractable: " << extracted;
  }
  else if (argv[1] == std::string("3")) {
    std::cout << "Distort images" << std::endl;

    // opencv refrac params 0.0150194    -0.0661002   0.00418932   0.00345423
    auto camera = CameraModel::InitCamera(CameraModelType::OpenCVCameraModel, {1920, 1080},
                                          {1279.4, 1279.4, 1920 / 2.0, 1080 / 2.0, -0.1, 0.01, 0.005, -0.005});
    cv::Mat map_x, map_y;
    CreateDistortMaps(&map_x, &map_y, camera);

    DistortImages(argv[2], argv[3], map_x, map_y);
  }
  else if (argv[1] == std::string("4")) {
    auto camera = CameraModel::InitCamera(CameraModelType::RadialCameraModel, {1280, 1024}, {1108.5, 640, 512, 0.5, 1});
    cv::Mat map_x, map_y;
    CreateDistortMaps(&map_x, &map_y, camera);
    std::tuple<cv::Mat&, cv::Mat&> maps(map_x, map_y);

    ProcessImages(argv[2], argv[3], 30, maps, 10);
  }
  else {
    std::cout << "Unkown code: " << argv[1];
  }
}