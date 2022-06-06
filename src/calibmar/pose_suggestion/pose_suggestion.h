#include "calibmar/core/calibration.h"

#include <Eigen/SparseCore>

namespace calibmar {
  namespace pose_suggestion {
    struct CostFunctionData {
      const calibmar::Calibration& calibration;
      Eigen::MatrixXd& jacobian_extendable;
      Eigen::SparseMatrix<double>& autocorrelation_matrix_extendable;
      std::pair<int, int>& pattern_cols_rows;
    };

    void ComputeJacobian(const std::vector<Image>& images, const std::vector<Eigen::Vector3d> points3D,
                         const colmap::Camera& camera, Eigen::MatrixXd& jacobian_intrinsics,
                         Eigen::MatrixXd& jacobian_extrinsics);

    void ComputeJacobian(Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, const std::vector<Eigen::Vector3d> points3D,
                         const colmap::Camera& camera, Eigen::MatrixXd& jacobian_intrinsics,
                         Eigen::MatrixXd& jacobian_extrinsics);

    double CostFunction(const std::vector<double>& x, CostFunctionData* data);

    // Estimates an optimal pose for the next calibration target image.
    // Pose values are given in the camera coordinate system.
    //
    // Implementation is based on "Efficient Pose Selection for Interactive Camera Calibration"
    // by Rojtberg, Pavel & Kuijper, Arjan for the initial two poses. The main pose suggestion procedure
    // is based on "Calibration Wizard: A Guidance System for Camera Calibration Based on Modelling Geometric and Corner
    // Uncertainty" by Peng, Songyou & Sturm, Peter.
    //
    // @param calibration Calibration, containing the current camera hypothesis and images that were used for calibration.
    // @param columns_points Columns of the calibration target. Must match shape of the image points.
    // @param rows_points Rows of the calibration target. Must match shape of the image points.
    // @param square_size Square size of the calibration target.
    // @param next_rotation Output rotation of the suggested next pose as quaternion (qw, qx, qy, qz).
    // @param next_translation Output translation of the suggested next pose.
    void EstimateNextBestPose(const Calibration& calibration, int columns_points, int rows_points, double square_size,
                              Eigen::Vector4d& next_rotation, Eigen::Vector3d& next_translation);

    // Computes the corner uncertainty auto correlation matrix from image points. It uses a pre calculated mapping
    // of corner angles to uncertainty value taken from the reference implementation of the calibration wizard paper.
    //
    // @param points2D Image points of projected chessboard corners. May contain multiple views.
    // @param views Number of views for points2D
    // @param pattern_cols_rows Columns and rows of the chessboard corners. Must match points2D shape.
    // @param mat The resulting auto correlation matrix. Highly sparse, only the needed values are inserted. Should be atleast
    //        of size (cols * rows * 2 * views) in each dimension.
    void ComputeCornerUncertaintyAutoCorrMat(const std::vector<Eigen::Vector2d>& points2D, int views,
                                             const std::pair<int, int>& pattern_cols_rows, Eigen::SparseMatrix<double>& mat);
  }
}