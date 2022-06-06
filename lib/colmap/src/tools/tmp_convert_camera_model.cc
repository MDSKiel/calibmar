#include <iostream>

#include <base/camera.h>
#include <base/camera_models.h>
#include <base/cost_functions.h>
#include <base/projection.h>
#include <util/math.h>
#include <util/misc.h>

using namespace colmap;

void ReadCalibFile(const std::string& path,
                   std::vector<Eigen::Vector2d>& points2D,
                   std::vector<Eigen::Vector3d>& points3D);

void OptimizeCameraModel(const std::vector<Eigen::Vector2d>& points2D,
                         const std::vector<Eigen::Vector3d>& points3D,
                         colmap::Camera& camera) {
  CHECK(camera.VerifyParams());

  ceres::Problem problem;
  ceres::Solver::Options solver_options;

  solver_options.minimizer_progress_to_stdout = true;
  solver_options.max_num_iterations = 300;
  solver_options.function_tolerance = solver_options.function_tolerance / 1e4;
  solver_options.gradient_tolerance = solver_options.gradient_tolerance / 1e4;
  solver_options.linear_solver_type = ceres::DENSE_QR;

  const size_t num_constraints = points2D.size();

  const Eigen::Vector4d qvec(1.0, 0.0, 0.0, 0.0);
  const Eigen::Vector3d tvec(0.0, 0.0, 0.0);

  double* camera_params = camera.ParamsData();

  for (size_t i = 0; i < num_constraints; i++) {
    const Eigen::Vector2d& point2D = points2D[i];
    const Eigen::Vector3d& point3D = points3D[i];

    ceres::CostFunction* cost_function =
        BundleAdjustmentConstantPoseConstantPoint3DCostFunction<
            MetashapeFisheyeCameraModel>::Create(point2D, point3D, qvec, tvec);

    problem.AddResidualBlock(cost_function, nullptr, camera_params);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
}

int main(int argc, char* argv[]) {
  std::cout << "Hello World" << std::endl;

  const std::string calib_file =
      "/home/mshe/workspace/tmp/lasse_convert_to_fisheye/calibfile.txt";

  std::vector<Eigen::Vector2d> points2D;
  std::vector<Eigen::Vector3d> points3D;
  ReadCalibFile(calib_file, points2D, points3D);

  colmap::Camera camera;
  size_t width = 5000;
  size_t height = 4000;
  camera.SetWidth(5000);
  camera.SetHeight(4000);

  camera.SetModelIdFromName("METASHAPE_FISHEYE");

  double fx = 2000;
  double fy = 2000;
  double cx = static_cast<double>(width) / 2.0;
  double cy = static_cast<double>(height) / 2.0;

  std::vector<double> camera_params(camera.NumParams(), 0.0);
  camera_params[0] = fx;
  camera_params[1] = fy;
  camera_params[2] = cx;
  camera_params[3] = cy;

  camera.SetParams(camera_params);

  std::cout << "Camera model: " << camera.ModelName()
            << "\n Params: " << camera.ParamsToString() << std::endl;

  OptimizeCameraModel(points2D, points3D, camera);

  std::cout << "Optimization finished, current params: \n"
            << camera.ParamsToString() << std::endl;

  std::vector<double> reprojection_errors;
  for (size_t i = 0; i < points2D.size(); i++) {
    double err = CalculateSquaredReprojectionError(
        points2D[i], points3D[i], Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
        Eigen::Vector3d::Zero(), camera);
    reprojection_errors.push_back(err);
  }
  double avg_err = Mean(reprojection_errors);
  double std_err = StdDev(reprojection_errors);

  std::cout << "Avg. / Std. error: " << avg_err << " / " << std_err
            << " pixels." << std::endl;
  return 0;
}

void ReadCalibFile(const std::string& path,
                   std::vector<Eigen::Vector2d>& points2D,
                   std::vector<Eigen::Vector3d>& points3D) {
  std::ifstream file(path, std::ios_base::in);
  CHECK(file.is_open());

  points2D.clear();
  points3D.clear();

  std::string line;

  // Skip the header.
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::stringstream line_stream = std::stringstream(line);
    std::vector<std::string> items;
    items.resize(5);
    for (size_t i = 0; i < 5; i++) {
      std::getline(line_stream, items[i], ' ');
    }
    Eigen::Vector2d point2D(std::stold(items[0]), std::stold(items[1]));
    Eigen::Vector3d point3D(std::stold(items[2]), std::stold(items[3]),
                            std::stold(items[4]));
    points2D.emplace_back(point2D);
    points3D.emplace_back(point3D);
  }

  std::cout << "Read " << points2D.size() << " points." << std::endl;
}