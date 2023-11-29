#include "colmap/estimators/cost_functions.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/math/random.h"
#include "colmap/scene/database_cache.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/sensor/models_refrac.h"
#include "colmap/util/logging.h"

using namespace colmap;

int main(int argc, char* argv[]) {
  if (false) {
    // For David, add additional images to the reconstruction.
    const std::string input_path =
        "/data2/mshe/omv_src/colmap-project/dataset/2023-08_AL-Daycruise/"
        "2023-08-10_Alkor_0001_GMR_PFM-109_AUV-LUISE_Mission-305/"
        "reconstruction_subset/result/exp0/sparse/0/";
    const std::string database_path =
        "/data2/mshe/omv_src/colmap-project/dataset/2023-08_AL-Daycruise/"
        "2023-08-10_Alkor_0001_GMR_PFM-109_AUV-LUISE_Mission-305/"
        "reconstruction_subset/result/database.db";
    const std::string output_path =
        "/data2/mshe/omv_src/colmap-project/dataset/2023-08_AL-Daycruise/"
        "2023-08-10_Alkor_0001_GMR_PFM-109_AUV-LUISE_Mission-305/"
        "reconstruction_subset/result/exp0/for_david/sparse/";

    Reconstruction recon;
    recon.Read(input_path);

    Database database(database_path);

    std::vector<Image> images = database.ReadAllImages();

    std::cout << "Current reconstruction: " << std::endl;
    std::cout << "Number of registered images: " << recon.NumRegImages()
              << std::endl;

    // Extra images I want to manually add:
    // std::unordered_set<image_t> extra_image_ids = {
    //     1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,
    //     14,  15,  16,  17,  18,  290, 291, 292, 293, 294, 295, 296, 297,
    //     298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308};

    std::unordered_set<image_t> extra_image_ids = {
        352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365,
        366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379,
        380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393};

    // extra_image_ids = {};
    std::cout << "Extra image ids: ";
    for (image_t id : extra_image_ids) {
      std::cout << id << " ";
    }
    std::cout << std::endl;

    // Take the absolute navigation pose and register them in the
    // reconstruction.

    // for (const auto image_id : extra_image_ids) {
    //   Image image_db = database.ReadImage(image_id);

    //   const Rigid3d prior_from_world = image_db.CamFromWorldPrior();
    //   const Rigid3d cam_from_world_prior = cam_from_prior * prior_from_world;

    //   image_db.CamFromWorld() = cam_from_world_prior;
    //   image_db.SetRegistered(true);
    //   recon.AddImage(image_db);
    // }

    // Take the relative navigation pose and register them in the
    // reconstruction.

    while (!extra_image_ids.empty()) {
      image_t reg_image_id = 0;
      image_t ref_image_id = 0;
      for (image_t image_id : extra_image_ids) {
        if (recon.ExistsImage(image_id - 1) &&
            recon.IsImageRegistered(image_id - 1)) {
          ref_image_id = image_id - 1;
          reg_image_id = image_id;
          break;
        }
        if (recon.ExistsImage(image_id + 1) &&
            recon.IsImageRegistered(image_id + 1)) {
          ref_image_id = image_id + 1;
          reg_image_id = image_id;
          break;
        }
      }
      if (reg_image_id == ref_image_id || reg_image_id == 0 ||
          ref_image_id == 0) {
        std::cout << "No neighbor images found in the reconstruction"
                  << std::endl;
      } else {
        std::cout << "Registering image " << reg_image_id
                  << ", the reference image is: " << ref_image_id << std::endl;

        Image image_reg_db = database.ReadImage(reg_image_id);
        Image image_ref_db = database.ReadImage(ref_image_id);
        const Image& image_ref = recon.Image(ref_image_id);

        const Rigid3d& reg_prior_from_world = image_reg_db.CamFromWorldPrior();
        const Rigid3d& ref_prior_from_world = image_ref_db.CamFromWorldPrior();

        const Rigid3d reg_from_ref =
            reg_prior_from_world * Inverse(ref_prior_from_world);

        const Rigid3d& ref_cam_from_world = image_ref.CamFromWorld();

        image_reg_db.CamFromWorld() = reg_from_ref * ref_cam_from_world;
        image_reg_db.SetRegistered(true);
        recon.AddImage(image_reg_db);
        recon.RegisterImage(reg_image_id);

        std::cout << "Number of registered images: " << recon.NumRegImages()
                  << std::endl;

        extra_image_ids.erase(reg_image_id);
      }
    }
    recon.WriteText(output_path);
  }

  if (false) {
    Camera camera;
    camera.SetWidth(2048);
    camera.SetHeight(1536);
    camera.SetModelIdFromName("PINHOLE");
    std::vector<double> params = {
        1300.900000, 1300.900000, 1024.000000, 768.000000};
    camera.SetParams(params);

    // Flatport setup.
    camera.SetRefracModelIdFromName("FLATPORT");
    Eigen::Vector3d int_normal;
    int_normal[0] = RandomUniformReal(-0.3, 0.3);
    int_normal[1] = RandomUniformReal(-0.3, 0.3);
    int_normal[2] = RandomUniformReal(0.7, 1.3);

    int_normal.normalize();

    // int_normal = Eigen::Vector3d::UnitZ();

    std::vector<double> flatport_params = {int_normal[0],
                                           int_normal[1],
                                           int_normal[2],
                                           0.01,
                                           0.014,
                                           1.0,
                                           1.52,
                                           1.334};
    camera.SetRefracParams(flatport_params);

    for (int i = 0; i < 10; i++) {
      const double x = RandomUniformReal(
          0.5, 10.0 /*static_cast<double>(camera.Width()) - 0.5*/);
      const double y = RandomUniformReal(
          0.5, 10.0 /*static_cast<double>(camera.Height()) - 0.5*/);

      Eigen::Vector2d point2D(x, y);
      Ray3D ray_refrac = camera.CamFromImgRefrac(point2D);
      const Eigen::Vector3d virtual_cam_center =
          camera.VirtualCameraCenter(ray_refrac);

      std::cout << "point: " << point2D.transpose()
                << ", virtual cam center: " << virtual_cam_center.transpose()
                << std::endl;
    }
  }

  if (false) {
    Eigen::Vector3d int_normal;
    int_normal[0] = RandomUniformReal(-0.3, 0.3);
    int_normal[1] = RandomUniformReal(-0.3, 0.3);
    int_normal[2] = RandomUniformReal(-0.7, 1.3);
    int_normal.normalize();
    std::cout << "int nromal: " << int_normal.transpose() << std::endl;
  }

  if (false) {
    // Create a reconstruction directly from navigation
    const std::string database_path =
        "/data2/mshe/omv_src/colmap-project/dataset/2023-08_AL-Daycruise/"
        "2023-08-10_Alkor_0001_GMR_PFM-109_AUV-LUISE_Mission-307/"
        "reconstruction/result/database.db";
    const std::string output_path =
        "/data2/mshe/omv_src/colmap-project/dataset/2023-08_AL-Daycruise/"
        "2023-08-10_Alkor_0001_GMR_PFM-109_AUV-LUISE_Mission-307/"
        "reconstruction/result/navigation/";

    Rigid3d prior_from_cam(
        Eigen::Quaterniond(0.711987, -0.00218027, -0.00757204, 0.702149),
        Eigen::Vector3d(0.347714, 0.0330715, -0.529309));

    Rigid3d cam_from_prior = Inverse(prior_from_cam);

    Database database(database_path);

    std::shared_ptr<DatabaseCache> database_cache =
        DatabaseCache::Create(database, 0, true, {});

    Reconstruction reconstruction;
    reconstruction.Load(*database_cache.get());

    for (const auto& image_it : reconstruction.Images()) {
      Image& image = reconstruction.Image(image_it.first);

      image.CamFromWorld() = cam_from_prior * image.CamFromWorldPrior();

      reconstruction.RegisterImage(image_it.first);
    }
    reconstruction.Write(output_path);
  }

  if (false) {
    std::cout << std::numeric_limits<double>::epsilon() << std::endl;
  }
  if (true) {
    // Check projection at the image boundary.
    // Setup parameters
    const size_t width = 1000;
    const size_t height = 1000;
    const double fov = 90;

    const double focal_length = 1000.0;

    std::vector<double> cam_params = {focal_length,
                                      static_cast<double>(width) * .5,
                                      static_cast<double>(height) * .5};

    // Randomly generate a rotation around the normal

    Eigen::Vector3d int_normal;
    int_normal[0] = colmap::RandomUniformReal(-0.3, 0.3);
    int_normal[1] = colmap::RandomUniformReal(-0.3, 0.3);
    int_normal[2] = colmap::RandomUniformReal(0.7, 1.3);

    int_normal = Eigen::Vector3d::UnitZ();
    int_normal.normalize();

    std::vector<double> flatport_params = {int_normal[0],
                                           int_normal[1],
                                           int_normal[2],
                                           -0.05,
                                           0.007,
                                           1.0,
                                           1.52,
                                           1.33};

    colmap::Camera camera;
    camera.SetWidth(width);
    camera.SetHeight(height);
    camera.SetModelIdFromName("SIMPLE_PINHOLE");
    camera.SetParams(cam_params);

    camera.SetRefracModelIdFromName("FLATPORT");
    camera.SetRefracParams(flatport_params);

    Eigen::Vector2d point1(556.0, 798.0);
    double d1 = 1.75;

    Eigen::Vector3d point3D_cam1 = camera.CamFromImgRefracPoint(point1, d1);

    std::cout << "Point3D cam1: " << point3D_cam1.transpose() << std::endl;

    Eigen::Vector2d proj1 = camera.ImgFromCamRefrac(Eigen::Vector3d(-0.3, -0.3, 0.5));

    Eigen::Vector2d point2(0.0, 789.235);
    double d2 = 1.75;

    Eigen::Vector3d point3D_cam2 = camera.CamFromImgRefracPoint(point2, d2);

    std::cout << "Point3D cam2: " << point3D_cam2.transpose() << std::endl;

    Eigen::Vector2d proj2 = camera.ImgFromCamRefrac(point3D_cam2);

    std::cout << "point1: " << point1.transpose()
              << " , projection1: " << proj1.transpose() << std::endl;
    std::cout << "point2: " << point2.transpose()
              << " , projection2: " << proj2.transpose() << std::endl;
  }

  if (false) {
    SetPRNGSeed(time(NULL));
    Camera camera;
    camera.SetWidth(2048);
    camera.SetHeight(1536);
    camera.SetModelIdFromName("PINHOLE");
    std::vector<double> params = {
        1300.900000, 1300.900000, 1024.000000, 768.000000};
    camera.SetParams(params);

    bool flatport = true;

    if (flatport) {
      // Flatport setup.
      camera.SetRefracModelIdFromName("FLATPORT");
      Eigen::Vector3d int_normal;
      int_normal[0] = RandomUniformReal(-0.3, 0.3);
      int_normal[1] = RandomUniformReal(-0.3, 0.3);
      int_normal[2] = RandomUniformReal(0.3, 2.3);

      int_normal.normalize();

      std::vector<double> flatport_params_gt = {int_normal[0],
                                                int_normal[1],
                                                int_normal[2],
                                                -0.04,
                                                0.007,
                                                1.0,
                                                1.52,
                                                1.334};
      camera.SetRefracParams(flatport_params_gt);
    } else {
      // Domeport setup.
      camera.SetRefracModelIdFromName("DOMEPORT");
      Eigen::Vector3d decentering;
      decentering[0] = RandomUniformReal(-0.03, 0.03);
      decentering[1] = RandomUniformReal(-0.001, 0.001);
      decentering[2] = RandomUniformReal(-0.001, 0.001);
      // decentering[0] = 0.0;
      // decentering[1] = 0.03;
      // decentering[2] = 0.00;

      std::vector<double> domeport_params_gt = {decentering[0],
                                                decentering[1],
                                                decentering[2],
                                                0.05,
                                                0.007,
                                                1.0,
                                                1.52,
                                                1.334};
      camera.SetRefracParams(domeport_params_gt);
    }

    std::cout << "GT refrac params: " << std::endl;
    std::cout << camera.RefracParamsToString() << std::endl;

    std::vector<Eigen::Vector2d> points2D;
    std::vector<Eigen::Vector2d> points2D_refrac;
    std::vector<Eigen::Vector3d> points3D;

    size_t num_points = 25;

    const double qx = RandomUniformReal(0.0, 1.0);
    const double tx = RandomUniformReal(0.0, 1.0);

    const Rigid3d cam_from_world_gt(
        Eigen::Quaterniond(1.0, qx, 0.0, 0.0).normalized(),
        Eigen::Vector3d(tx, 0.0, 0.0));

    for (size_t i = 0; i < num_points; i++) {
      Eigen::Vector2d point2D_refrac;
      point2D_refrac.x() =
          RandomUniformReal(0.5, static_cast<double>(camera.Width()) - 0.5);
      point2D_refrac.y() =
          RandomUniformReal(0.5, static_cast<double>(camera.Height()) - 0.5);

      const double depth = RandomUniformReal(0.5, 10.0);
      const Eigen::Vector3d point3D_local =
          camera.CamFromImgRefracPoint(point2D_refrac, depth);

      const Eigen::Vector3d point3D_world =
          Inverse(cam_from_world_gt) * point3D_local;

      // Now project the point in-air.
      Eigen::Vector2d point2D = camera.ImgFromCam(point3D_local.hnormalized());

      // point2D_refrac(0) += RandomGaussian(0.0, 3.0);
      // point2D_refrac(1) += RandomGaussian(0.0, 3.0);

      points2D.push_back(point2D);
      points2D_refrac.push_back(point2D_refrac);
      points3D.push_back(point3D_world);
    }

    ceres::Problem problem;

    if (flatport) {
      std::vector<double> flatport_params = camera.RefracParams();
      flatport_params[0] = 0.0;
      flatport_params[1] = 0.0;
      flatport_params[2] = 1.0;
      flatport_params[3] = 0.005;
      camera.SetRefracParams(flatport_params);
    } else {
      std::vector<double> domeport_params = camera.RefracParams();
      domeport_params[0] = 0.00;
      domeport_params[1] = 0.00;
      domeport_params[2] = 0.002;
      camera.SetRefracParams(domeport_params);
    }

    std::cout << "Initial refrac params: " << std::endl;
    std::cout << camera.RefracParamsToString() << std::endl;

    for (size_t i = 0; i < num_points; i++) {
      const Eigen::Vector2d& point2D_refrac = points2D_refrac[i];
      Eigen::Vector3d& point3D = points3D[i];

      ceres::CostFunction* cost_function;
      if (flatport) {
        cost_function = ReprojErrorRefracConstantPoseCostFunction<
            FlatPort,
            PinholeCameraModel>::Create(cam_from_world_gt, point2D_refrac);
      } else {
        cost_function = ReprojErrorRefracConstantPoseCostFunction<
            DomePort,
            PinholeCameraModel>::Create(cam_from_world_gt, point2D_refrac);
      }
      problem.AddResidualBlock(cost_function,
                               nullptr,
                               point3D.data(),
                               camera.ParamsData(),
                               camera.RefracParamsData());
      problem.SetParameterBlockConstant(point3D.data());
    }

    problem.SetParameterBlockConstant(camera.ParamsData());

    std::vector<int> refrac_params_idxs(camera.NumRefracParams());
    std::iota(refrac_params_idxs.begin(), refrac_params_idxs.end(), 0);

    const std::vector<size_t>& optimizable_refrac_params_idxs =
        camera.OptimizableRefracParamsIdxs();

    std::vector<int> const_params_idxs;
    std::set_difference(refrac_params_idxs.begin(),
                        refrac_params_idxs.end(),
                        optimizable_refrac_params_idxs.begin(),
                        optimizable_refrac_params_idxs.end(),
                        std::back_inserter(const_params_idxs));
    if (flatport) {
      for (int& idx : const_params_idxs) {
        idx -= 3;
      }
      ceres::SphereManifold<3> sphere_manifold = ceres::SphereManifold<3>();
      ceres::SubsetManifold subset_manifold = ceres::SubsetManifold(
          camera.NumRefracParams() - 3, const_params_idxs);
      ceres::ProductManifold<ceres::SphereManifold<3>, ceres::SubsetManifold>*
          product_manifold =
              new ceres::ProductManifold<ceres::SphereManifold<3>,
                                         ceres::SubsetManifold>(
                  sphere_manifold, subset_manifold);
      problem.SetManifold(camera.RefracParamsData(), product_manifold);
    } else {
      SetSubsetManifold(static_cast<int>(camera.NumRefracParams()),
                        const_params_idxs,
                        &problem,
                        camera.RefracParamsData());
    }
    ceres::Solver::Options solver_options;
    ceres::Solver::Summary summary;

    solver_options.function_tolerance = 1e-20;
    solver_options.gradient_tolerance = 1e-20;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.minimizer_progress_to_stdout = true;

    ceres::Solve(solver_options, &problem, &summary);

    std::cout << "Optimized refrac params: " << std::endl;
    std::cout << camera.RefracParamsToString() << std::endl;
  }

  if(false){
    // Given a camera setup, check for total reflection.
    Camera camera;
    camera.SetWidth(1113);
    camera.SetHeight(835);
    camera.SetModelIdFromName("PINHOLE");
    std::vector<double> params = {
        100.476237, 100.476237, 556.500000, 417.500000};
    camera.SetParams(params);

    camera.SetRefracModelIdFromName("FLATPORT");
    std::vector<double> refrac_params = {0.000000, 0.000000, 1.000000, 0.050000, 0.007000, 1.000000, 1.520000, 1.330000};
    camera.SetRefracParams(refrac_params);

    Eigen::Vector2d p1(0.0, 0.0);
    Eigen::Vector2d p2(static_cast<double>(camera.Width()) + 2000.0, camera.Height());

    //Ray3D ray1 = camera.CamFromImgRefrac(p1);
    Ray3D ray2 = camera.CamFromImgRefrac(p2);

    //std::cout << "ray1: " << ray1.dir.transpose() << std::endl;
    std::cout << "ray2: " << ray2.dir.transpose() << std::endl;
  }

  if(false){
    Eigen::Quaterniond rot(1.2, 0.1, 0.2, 0.3);

    std::cout << rot.coeffs() << std::endl;
  }

  return true;
}