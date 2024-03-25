#include "colmap/estimators/refrac_relative_pose.h"

#include "colmap/geometry/essential_matrix.h"
#include "colmap/geometry/pose.h"
#include "colmap/geometry/triangulation.h"
#include "colmap/util/eigen_alignment.h"
#include "colmap/util/logging.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

namespace colmap {
void RefracRelPoseEstimator::Estimate(const std::vector<X_t>& points1,
                                      const std::vector<Y_t>& points2,
                                      std::vector<M_t>* models) {
  CHECK_GE(points1.size(), 0);
  CHECK_EQ(points1.size(), points2.size());
  CHECK(models != nullptr);

  models->clear();

  // Compute virtual camera centers.
  const size_t kNumPoints = points1.size();
  std::vector<Eigen::Vector3d> virtual_proj_centers1;
  std::vector<Eigen::Vector3d> virtual_proj_centers2;
  virtual_proj_centers1.reserve(kNumPoints);
  virtual_proj_centers2.reserve(kNumPoints);

  for (size_t i = 0; i < kNumPoints; ++i) {
    virtual_proj_centers1.push_back(
        points1[i].virtual_from_real.rotation.inverse() *
        -points1[i].virtual_from_real.translation);
    virtual_proj_centers2.push_back(
        points2[i].virtual_from_real.rotation.inverse() *
        -points2[i].virtual_from_real.translation);
  }

  // Compose A matrix, see (4.19) on page 9
  Eigen::MatrixXd A(kNumPoints, 18);
  A.setZero();

  for (size_t i = 0; i < kNumPoints; ++i) {
    const Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d& r1 = points1[i].ray_in_virtual;
    const Eigen::Matrix<double, 1, 3> r1_t =
        points1[i].ray_in_virtual.transpose();
    const Eigen::Matrix<double, 1, 3> r2_t =
        points2[i].ray_in_virtual.transpose();

    const Eigen::Vector3d& vc1 = virtual_proj_centers1[i];
    const Eigen::Vector3d& vc2 = virtual_proj_centers2[i];
    const Eigen::Matrix3d vc1_hat = CrossProductMatrix(vc1);
    const Eigen::Matrix3d vc2_hat = CrossProductMatrix(vc2);

    // Cache the first kronecker product.
    const Eigen::KroneckerProduct<Eigen::Matrix<double, 1, 3>,
                                  Eigen::Matrix<double, 3, 3>>
        kron1 =
            Eigen::KroneckerProduct<Eigen::Matrix<double, 1, 3>,
                                    Eigen::Matrix<double, 3, 3>>(r1_t, eye3);

    // Cache the second kronecker product.
    const Eigen::Matrix<double, 1, 3> cv1_u = (vc1_hat * r1).transpose();
    const Eigen::KroneckerProduct<Eigen::Matrix<double, 1, 3>,
                                  Eigen::Matrix<double, 3, 3>>
        kron2 =
            Eigen::KroneckerProduct<Eigen::Matrix<double, 1, 3>,
                                    Eigen::Matrix<double, 3, 3>>(cv1_u, eye3);

    A.block<1, 9>(i, 0) = r2_t * vc2_hat * kron1 - r2_t * kron2;
    A.block<1, 9>(i, 9) = -r2_t * kron1;
  }

  Eigen::MatrixXd AR = A.block(0, 0, kNumPoints, 9);
  Eigen::MatrixXd AE = A.block(0, 9, kNumPoints, 9);
  Eigen::MatrixXd ARpinv = AR.completeOrthogonalDecomposition().pseudoInverse();
  // Eigen::MatrixXd ARpinv = pseudoInverse(AR);
  Eigen::MatrixXd eye_N(kNumPoints, kNumPoints);
  eye_N.setIdentity();

  Eigen::MatrixXd B = (AR * ARpinv - eye_N) * AE;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeFullV);

  Eigen::MatrixXd sol = svd.matrixV().col(8);
  const Eigen::Map<const Eigen::Matrix3d> E_raw(sol.data());

  // Enforcing the internal constraint that two singular values must be equal
  // and one must be zero.
  Eigen::JacobiSVD<Eigen::Matrix3d> E_raw_svd(
      E_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular_values = E_raw_svd.singularValues();
  singular_values(0) = (singular_values(0) + singular_values(1)) / 2.0;
  singular_values(1) = singular_values(0);
  singular_values(2) = 0.0;
  const Eigen::Matrix3d E = E_raw_svd.matrixU() * singular_values.asDiagonal() *
                            E_raw_svd.matrixV().transpose();

  // Solve kernel part (Basically repeat PoseFromEssentialMatrix, but replace
  // proj_center by virtual_proj_center).
  Eigen::Matrix3d R_cam1_from_cam2;
  Eigen::Vector3d t_cam1_from_cam2;
  std::vector<Eigen::Vector3d> points3D;
  Eigen::Matrix3d R1;
  Eigen::Matrix3d R2;
  Eigen::Vector3d t;
  DecomposeEssentialMatrix(E, &R1, &R2, &t);
  points3D.clear();

  // Generate all possible projection matrix combinations.
  const std::array<Eigen::Matrix3d, 4> R_cmbs{{R1, R2, R1, R2}};
  const std::array<Eigen::Vector3d, 4> t_cmbs{{t, t, -t, -t}};

  auto CalculateDepth = [](const Eigen::Matrix3x4d& cam_from_world,
                           const Eigen::Vector3d& point3D) {
    const double proj_z = cam_from_world.row(2).dot(point3D.homogeneous());
    return proj_z * cam_from_world.col(2).norm();
  };

  for (size_t i = 0; i < R_cmbs.size(); ++i) {
    std::vector<Eigen::Vector3d> points3D_cmb;
    // Check cheriality here

    const double kMinDepth = std::numeric_limits<double>::epsilon();
    const double max_depth =
        1000.0f * (R_cmbs[i].transpose() * t_cmbs[i]).norm();

    for (size_t j = 0; j < kNumPoints; ++j) {
      const Rigid3d& virtual_from_real1 = points1[j].virtual_from_real;
      const Rigid3d& virtual_from_real2 = points2[j].virtual_from_real;

      const Rigid3d world_from_cam2(Eigen::Quaterniond(R_cmbs[i].transpose()),
                                    t_cmbs[i]);
      const Rigid3d cam2_from_world = Inverse(world_from_cam2);
      const Rigid3d virtual2_from_world = virtual_from_real2 * cam2_from_world;

      const Eigen::Matrix3x4d virtual_proj_matrix1 =
          virtual_from_real1.ToMatrix();
      const Eigen::Matrix3x4d virtual_proj_matrix2 =
          virtual2_from_world.ToMatrix();

      const Eigen::Vector3d point3D =
          TriangulatePoint(virtual_proj_matrix1,
                           virtual_proj_matrix2,
                           points1[j].ray_in_virtual.hnormalized(),
                           points2[j].ray_in_virtual.hnormalized());

      const double depth1 = CalculateDepth(virtual_proj_matrix1, point3D);
      if (depth1 > kMinDepth && depth1 < max_depth) {
        const double depth2 = CalculateDepth(virtual_proj_matrix2, point3D);
        if (depth2 > kMinDepth && depth2 < max_depth) {
          points3D_cmb.push_back(point3D);
        }
      }
    }

    if (points3D_cmb.size() >= points3D.size()) {
      R_cam1_from_cam2 = R_cmbs[i].transpose();
      t_cam1_from_cam2 = t_cmbs[i];
      points3D = points3D_cmb;
    }
  }

  // Solve for t.

  A.resize(kNumPoints, 3);
  A.setZero();

  Eigen::VectorXd b(kNumPoints);
  b.setZero();

  for (size_t i = 0; i < kNumPoints; ++i) {
    A.row(i) = points2[i].ray_in_virtual.transpose() *
               R_cam1_from_cam2.transpose() *
               CrossProductMatrix(points1[i].ray_in_virtual);

    b.row(i) = points2[i].ray_in_virtual.transpose() *
                   CrossProductMatrix(virtual_proj_centers2[i]) *
                   R_cam1_from_cam2.transpose() * points1[i].ray_in_virtual -
               points2[i].ray_in_virtual.transpose() *
                   R_cam1_from_cam2.transpose() *
                   CrossProductMatrix(virtual_proj_centers1[i]) *
                   points1[i].ray_in_virtual;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> t_svd(
      A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  t_cam1_from_cam2 = t_svd.solve(b);

  models->push_back(
      Inverse(Rigid3d(Eigen::Quaterniond(R_cam1_from_cam2), t_cam1_from_cam2)));
}

void RefracRelPoseEstimator::Residuals(const std::vector<X_t>& points1,
                                       const std::vector<Y_t>& points2,
                                       const M_t& cam2_from_cam1,
                                       std::vector<double>* residuals) {
  CHECK_EQ(points1.size(), points2.size());
  residuals->resize(points1.size(), 0);
  for (size_t i = 0; i < points1.size(); ++i) {
    const Rigid3d v2_from_v1 =
        points2[i].virtual_from_real *
        (cam2_from_cam1 * Inverse(points1[i].virtual_from_real));
    const Eigen::Matrix3d E = EssentialMatrixFromPose(v2_from_v1);
    const Eigen::Vector3d Ex1 =
        E * points1[i].ray_in_virtual.hnormalized().homogeneous();
    const Eigen::Vector3d x2 =
        points2[i].ray_in_virtual.hnormalized().homogeneous();
    const Eigen::Vector3d Etx2 = E.transpose() * x2;
    const double x2tEx1 = x2.transpose() * Ex1;
    (*residuals)[i] = x2tEx1 * x2tEx1 /
                      (Ex1(0) * Ex1(0) + Ex1(1) * Ex1(1) + Etx2(0) * Etx2(0) +
                       Etx2(1) * Etx2(1));
  }
}

}  // namespace colmap
