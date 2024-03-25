#include "colmap/math/random.h"
#include "colmap/scene/camera.h"
#include "colmap/util/logging.h"

#include <iostream>

using namespace colmap;

int main(int argc, char** argv) {
  // Create a traditional in-air camera.

  Camera camera;
  camera.width = 1920;
  camera.height = 1080;
  camera.model_id = CameraModelId::kSimpleRadial;
  std::vector<double> params = {
      1297.3655404279762, 1297.3655404279762, 960.0, 540.0, 0.01};
  camera.params = params;

  // Set the camera as a refractive camera (flat-port case).
  camera.refrac_model_id = CameraRefracModelId::kFlatPort;
  Eigen::Vector3d int_normal;
  int_normal[0] = RandomUniformReal(-0.2, 0.2);
  int_normal[1] = RandomUniformReal(-0.2, 0.2);
  int_normal[2] = RandomUniformReal(0.8, 1.2);
  int_normal.normalize();

  std::vector<double> flatport_params = {int_normal[0],
                                         int_normal[1],
                                         int_normal[2],
                                         colmap::RandomUniformReal(0.001, 0.05),
                                         colmap::RandomUniformReal(0.002, 0.2),
                                         1.0,
                                         1.52,
                                         1.334};
  camera.refrac_params = flatport_params;

  // Given a random 2D image point, back-project it to 3D as a refracted ray in
  // the water.
  Eigen::Vector2d point2D(
      RandomUniformReal(0.5, static_cast<double>(camera.width - 0.5)),
      RandomUniformReal(0.5, static_cast<double>(camera.height - 0.5)));

  colmap::Ray3D ray_w = camera.CamFromImgRefrac(point2D);
  LOG(INFO) << "ray origin on the interface: " << ray_w.ori.transpose()
            << " , ray direction in the water: " << ray_w.dir.transpose();

  // Find a 3D point on this ray at a random depth.
  const double depth = RandomUniformReal(0.5, 10.0);
  Eigen::Vector3d point3D = ray_w.At(depth);

  // Forward project the 3D point onto the image refractively.
  Eigen::Vector2d projection_refrac = camera.ImgFromCamRefrac(point3D);

  LOG(INFO) << " Original 2D point: " << point2D.transpose();
  LOG(INFO) << "Projected 2D point: " << projection_refrac.transpose();
  return EXIT_SUCCESS;
}