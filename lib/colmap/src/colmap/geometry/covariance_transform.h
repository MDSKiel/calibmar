#pragma once

#include "colmap/geometry/rigid3.h"
#include "colmap/util/types.h"

#include <vector>

namespace colmap {

class UnscentedTransform {
 public:
  struct WeightedSigmaPoint {
    Eigen::VectorXd point;
    double weight_mean;
    double weight_cov;
  };

  explicit UnscentedTransform(const double alpha = 0.001,
                              const double beta = 2.0,
                              const double kappa = 0.0);

  inline void SetAlpha(const double alpha);
  inline void SetBeta(const double beta);
  inline void SetKappa(const double kappa);

 protected:
  virtual bool TransformPoint(const Eigen::VectorXd& src,
                              Eigen::VectorXd& dst) const = 0;

  bool ComputeSigmaPoints(const Eigen::VectorXd& src_mean,
                          const Eigen::MatrixXd& src_cov,
                          std::vector<WeightedSigmaPoint>& sigma_points) const;

  bool TransformImpl(const Eigen::VectorXd& src_mean,
                     const Eigen::MatrixXd& src_cov,
                     Eigen::VectorXd& dst_mean,
                     Eigen::MatrixXd& dst_cov) const;
  // The alpha parameter determines the spread of the sigma points
  double alpha_;

  // Beta is used to incorporate prior knowledge of the distribution of x.
  // For Gaussian distribution, beta = 2.0 is optimal.
  double beta_;

  // kappa is a secondary scaling parameter. In other papers from Uhlman kappa
  // is fixed to 1.0
  double kappa_;
};

// Transform covariance reprensented by Euler angle into quaternion
// representation. Euler angles are 3D vector with Rx, Ry, Rz in [rad]. The
// rotation order is ZYX: R = Rz * Ry * Rx;
class CovEulerZYXToQuaternion : public UnscentedTransform {
 public:
  explicit CovEulerZYXToQuaternion();
  bool Transform(const Eigen::Vector3d& euler_src,
                 const Eigen::Matrix3d& cov_src,
                 Eigen::Quaterniond& quat_dst,
                 Eigen::Matrix4d& cov_dst) const;

 protected:
  bool TransformPoint(const Eigen::VectorXd& src,
                      Eigen::VectorXd& dst) const override;
};

class CovQuaternionToEulerZYX : public UnscentedTransform {
 public:
  explicit CovQuaternionToEulerZYX();
  bool Transform(const Eigen::Quaterniond& quat_src,
                 const Eigen::Matrix4d& cov_src,
                 Eigen::Vector3d& euler_dst,
                 Eigen::Matrix3d& cov_dst) const;

 protected:
  bool TransformPoint(const Eigen::VectorXd& src,
                      Eigen::VectorXd& dst) const override;
};

class CovRigid3dInverse : public UnscentedTransform {
 public:
  explicit CovRigid3dInverse();

  bool Transform(const Rigid3d& tform_src,
                 const Eigen::Matrix7d& cov_src,
                 Rigid3d& tform_dst,
                 Eigen::Matrix7d& cov_dst) const;

 protected:
  bool TransformPoint(const Eigen::VectorXd& src,
                      Eigen::VectorXd& dst) const override;
};

class CovRigid3dTransform : public UnscentedTransform {
 public:
  explicit CovRigid3dTransform();

  bool Transform(const Rigid3d& tform_src,
                 const Eigen::Matrix7d& cov_src,
                 const Rigid3d& dst_from_src,
                 const Eigen::Matrix7d& cov_dst_from_src,
                 Rigid3d& tform_dst,
                 Eigen::Matrix7d& cov_dst) const;

 protected:
  bool TransformPoint(const Eigen::VectorXd& src,
                      Eigen::VectorXd& dst) const override;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

void UnscentedTransform::SetAlpha(const double alpha) { alpha_ = alpha; }

void UnscentedTransform::SetBeta(const double beta) { beta_ = beta; }

void UnscentedTransform::SetKappa(const double kappa) { kappa_ = kappa; }

};  // namespace colmap