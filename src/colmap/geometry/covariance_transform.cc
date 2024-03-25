#include "colmap/geometry/covariance_transform.h"

#include "colmap/geometry/pose.h"
#include "colmap/geometry/rigid3.h"

namespace colmap {

UnscentedTransform::UnscentedTransform(const double alpha,
                                       const double beta,
                                       const double kappa)
    : alpha_(alpha), beta_(beta), kappa_(kappa) {}

bool UnscentedTransform::ComputeSigmaPoints(
    const Eigen::VectorXd& src_mean,
    const Eigen::MatrixXd& src_cov,
    std::vector<WeightedSigmaPoint>& sigma_points) const {
  const size_t dim = src_mean.size();
  const double dbl_dim = static_cast<double>(dim);
  const double alpha2 = alpha_ * alpha_;
  const double lambda = alpha2 * (dbl_dim + kappa_) - dbl_dim;
  const double dim_plus_lambda = alpha2 * (dbl_dim + kappa_);

  sigma_points.resize(2 * dim + 1);

  // Numerically very unstable without normalization !!!
  double factor = 0.0;
  for (size_t i = 0; i < dim; i++) {
    factor += src_cov(i, i);
  }
  Eigen::MatrixXd normalized_cov = src_cov / factor;
  // Enforce positive definiteness by adding epislon to diagonal elements.
  const double eps_fac = 2.0 * std::numeric_limits<double>::epsilon();
  for (size_t i = 0; i < dim; i++) normalized_cov(i, i) += eps_fac;

  Eigen::MatrixXd sqrt_cov = normalized_cov.llt().matrixL();
  factor *= dim_plus_lambda;
  factor = sqrt(factor);

  // Compute sigma points.
  sigma_points[0].point = src_mean;
  sigma_points[0].weight_mean = lambda / dim_plus_lambda;
  sigma_points[0].weight_cov = lambda / dim_plus_lambda + 1.0 - alpha2 + beta_;

  size_t point_idx;
  for (size_t i = 0; i < dim; i++) {
    point_idx = i + 1;
    Eigen::VectorXd col = sqrt_cov.col(i);
    col *= factor;

    sigma_points[point_idx].point = src_mean + col;
    sigma_points[point_idx + dim].point = src_mean - col;
    sigma_points[point_idx].weight_mean =
        sigma_points[point_idx + dim].weight_mean =
            sigma_points[point_idx].weight_cov =
                sigma_points[point_idx + dim].weight_cov =
                    1.0 / (2.0 * dim_plus_lambda);
  }

  return true;
}

bool UnscentedTransform::TransformImpl(const Eigen::VectorXd& src_mean,
                                       const Eigen::MatrixXd& src_cov,
                                       Eigen::VectorXd& dst_mean,
                                       Eigen::MatrixXd& dst_cov) const {
  if (src_cov.trace() < std::numeric_limits<double>::epsilon()) {
    // Zero covariance matrix.
    if (!TransformPoint(src_mean, dst_mean)) {
      return false;
    }
    const size_t dim = dst_mean.size();
    dst_cov.setZero(dim, dim);
  } else {
    std::vector<WeightedSigmaPoint> sigma_points;

    // Check result dimension.
    if (!TransformPoint(src_mean, dst_mean)) {
      // Transformation of the mean failed, returning false.
      return false;
    }

    // Compute sigma points.
    if (!ComputeSigmaPoints(src_mean, src_cov, sigma_points)) return false;

    const size_t dim = dst_mean.size();
    // Initialize the result.
    dst_mean.setZero();
    dst_cov.setZero(dim, dim);

    // Transform sigma points and compute the new mean and new covariance.
    for (auto& sigma_point : sigma_points) {
      const Eigen::VectorXd src = sigma_point.point;
      if (!TransformPoint(src, sigma_point.point)) return false;
      // Compute the weighted mean.
      dst_mean += sigma_point.point * sigma_point.weight_mean;
    }

    for (auto& sigma_point : sigma_points) {
      // Compute the covariance.
      const Eigen::VectorXd diff = sigma_point.point - dst_mean;
      dst_cov += sigma_point.weight_cov * diff * diff.transpose();
    }

    // Finally check if the new covariance is valid.
    for (int i = 0; i < dst_cov.rows(); i++) {
      for (int j = 0; j < dst_cov.cols(); j++) {
        if (std::isnan(dst_cov(i, j)) || std::isnan(dst_cov(i, j))) {
          return false;
        }
      }
    }
  }
  return true;
}

CovEulerZYXToQuaternion::CovEulerZYXToQuaternion() : UnscentedTransform() {}

bool CovEulerZYXToQuaternion::Transform(const Eigen::Vector3d& euler_src,
                                        const Eigen::Matrix3d& cov_src,
                                        Eigen::Quaterniond& quat_dst,
                                        Eigen::Matrix4d& cov_dst) const {
  Eigen::VectorXd qvec_out;
  Eigen::MatrixXd cov_out;
  if (!TransformImpl(euler_src, cov_src, qvec_out, cov_out)) return false;
  quat_dst =
      Eigen::Quaterniond(qvec_out(0), qvec_out(1), qvec_out(2), qvec_out(3))
          .normalized();
  cov_dst = cov_out;
  return true;
}

bool CovEulerZYXToQuaternion::TransformPoint(const Eigen::VectorXd& src,
                                             Eigen::VectorXd& dst) const {
  dst.setZero(4);
  const Eigen::Matrix3d R = EulerAnglesToRotationMatrix(src(0), src(1), src(2));
  const Eigen::Quaterniond quat_out = Eigen::Quaterniond(R).normalized();

  dst(0) = quat_out.w();
  dst(1) = quat_out.x();
  dst(2) = quat_out.y();
  dst(3) = quat_out.z();
  return true;
}

CovQuaternionToEulerZYX::CovQuaternionToEulerZYX() : UnscentedTransform() {}

bool CovQuaternionToEulerZYX::Transform(const Eigen::Quaterniond& quat_src,
                                        const Eigen::Matrix4d& cov_src,
                                        Eigen::Vector3d& euler_dst,
                                        Eigen::Matrix3d& cov_dst) const {
  Eigen::Vector4d qvec_src(
      quat_src.w(), quat_src.x(), quat_src.y(), quat_src.z());
  Eigen::VectorXd euler_out;
  Eigen::MatrixXd cov_out;

  if (!TransformImpl(qvec_src, cov_src, euler_out, cov_out)) return false;
  euler_dst(0) = euler_out(0);
  euler_dst(1) = euler_out(1);
  euler_dst(2) = euler_out(2);
  cov_dst = cov_out;
  return true;
}

bool CovQuaternionToEulerZYX::TransformPoint(const Eigen::VectorXd& src,
                                             Eigen::VectorXd& dst) const {
  dst.setZero(3);
  const Eigen::Matrix3d R = Eigen::Quaterniond(src(0), src(1), src(2), src(3))
                                .normalized()
                                .toRotationMatrix();
  RotationMatrixToEulerAngles(R, &dst(0), &dst(1), &dst(2));
  return true;
}

CovRigid3dInverse::CovRigid3dInverse() : UnscentedTransform() {}

bool CovRigid3dInverse::Transform(const Rigid3d& tform_src,
                                  const Eigen::Matrix7d& cov_src,
                                  Rigid3d& tform_dst,
                                  Eigen::Matrix7d& cov_dst) const {
  Eigen::Matrix<double, 7, 1> tform_vec_src;
  tform_vec_src(0) = tform_src.rotation.w();
  tform_vec_src(1) = tform_src.rotation.x();
  tform_vec_src(2) = tform_src.rotation.y();
  tform_vec_src(3) = tform_src.rotation.z();
  tform_vec_src(4) = tform_src.translation.x();
  tform_vec_src(5) = tform_src.translation.y();
  tform_vec_src(6) = tform_src.translation.z();

  Eigen::VectorXd tform_vec_dst;
  Eigen::MatrixXd cov_out;
  tform_vec_dst.setZero(7);
  cov_out.setZero(7, 7);
  if (!TransformImpl(tform_vec_src, cov_src, tform_vec_dst, cov_out))
    return false;

  tform_dst.rotation = Eigen::Quaterniond(tform_vec_dst(0),
                                          tform_vec_dst(1),
                                          tform_vec_dst(2),
                                          tform_vec_dst(3))
                           .normalized();
  tform_dst.translation.x() = tform_vec_dst(4);
  tform_dst.translation.y() = tform_vec_dst(5);
  tform_dst.translation.z() = tform_vec_dst(6);
  cov_dst = cov_out;
  return true;
}

bool CovRigid3dInverse::TransformPoint(const Eigen::VectorXd& src,
                                       Eigen::VectorXd& dst) const {
  Rigid3d tform_src;
  tform_src.rotation =
      Eigen::Quaterniond(src(0), src(1), src(2), src(3)).normalized();
  tform_src.translation.x() = src(4);
  tform_src.translation.y() = src(5);
  tform_src.translation.z() = src(6);

  Rigid3d tform_dst = Inverse(tform_src);
  dst.setZero(7);
  dst(0) = tform_dst.rotation.w();
  dst(1) = tform_dst.rotation.x();
  dst(2) = tform_dst.rotation.y();
  dst(3) = tform_dst.rotation.z();
  dst(4) = tform_dst.translation.x();
  dst(5) = tform_dst.translation.y();
  dst(6) = tform_dst.translation.z();
  return true;
}

CovRigid3dTransform::CovRigid3dTransform() : UnscentedTransform() {}

bool CovRigid3dTransform::Transform(const Rigid3d& tform_src,
                                    const Eigen::Matrix7d& cov_src,
                                    const Rigid3d& dst_from_src,
                                    const Eigen::Matrix7d& cov_dst_from_src,
                                    Rigid3d& tform_dst,
                                    Eigen::Matrix7d& cov_dst) const {
  Eigen::Matrix<double, 14, 1> tform_vec_src;
  tform_vec_src(0) = tform_src.rotation.w();
  tform_vec_src(1) = tform_src.rotation.x();
  tform_vec_src(2) = tform_src.rotation.y();
  tform_vec_src(3) = tform_src.rotation.z();
  tform_vec_src(4) = tform_src.translation.x();
  tform_vec_src(5) = tform_src.translation.y();
  tform_vec_src(6) = tform_src.translation.z();
  tform_vec_src(7) = dst_from_src.rotation.w();
  tform_vec_src(8) = dst_from_src.rotation.x();
  tform_vec_src(9) = dst_from_src.rotation.y();
  tform_vec_src(10) = dst_from_src.rotation.z();
  tform_vec_src(11) = dst_from_src.translation.x();
  tform_vec_src(12) = dst_from_src.translation.y();
  tform_vec_src(13) = dst_from_src.translation.z();
  Eigen::Matrix<double, 14, 14> cov_in = Eigen::Matrix<double, 14, 14>::Zero();
  cov_in.block<7, 7>(0, 0) = cov_src;
  cov_in.block<7, 7>(7, 7) = cov_dst_from_src;

  Eigen::VectorXd tform_vec_dst;
  Eigen::MatrixXd cov_out;
  tform_vec_dst.setZero(7);
  cov_out.setZero(7, 7);
  if (!TransformImpl(tform_vec_src, cov_in, tform_vec_dst, cov_out))
    return false;
  tform_dst.rotation = Eigen::Quaterniond(tform_vec_dst(0),
                                          tform_vec_dst(1),
                                          tform_vec_dst(2),
                                          tform_vec_dst(3))
                           .normalized();
  tform_dst.translation.x() = tform_vec_dst(4);
  tform_dst.translation.y() = tform_vec_dst(5);
  tform_dst.translation.z() = tform_vec_dst(6);

  cov_dst = cov_out;
  return true;
}

bool CovRigid3dTransform::TransformPoint(const Eigen::VectorXd& src,
                                         Eigen::VectorXd& dst) const {
  Rigid3d tform_src, tform_dst_from_src;
  tform_src.rotation =
      Eigen::Quaterniond(src(0), src(1), src(2), src(3)).normalized();
  tform_src.translation.x() = src(4);
  tform_src.translation.y() = src(5);
  tform_src.translation.z() = src(6);
  tform_dst_from_src.rotation =
      Eigen::Quaterniond(src(7), src(8), src(9), src(10)).normalized();
  tform_dst_from_src.translation.x() = src(11);
  tform_dst_from_src.translation.y() = src(12);
  tform_dst_from_src.translation.z() = src(13);

  Rigid3d tform_dst = tform_dst_from_src * tform_src;

  dst.setZero(7);
  dst(0) = tform_dst.rotation.w();
  dst(1) = tform_dst.rotation.x();
  dst(2) = tform_dst.rotation.y();
  dst(3) = tform_dst.rotation.z();
  dst(4) = tform_dst.translation.x();
  dst(5) = tform_dst.translation.y();
  dst(6) = tform_dst.translation.z();
  return true;
}

}  // namespace colmap