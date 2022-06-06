#include <fstream>
#include <iostream>

#include "base/cost_functions.h"
#include "base/pose.h"

using namespace colmap;

struct Pose3D {
  Eigen::Vector4d qvec;
  Eigen::Vector3d tvec;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Constraint {
  int id_begin;
  int id_end;

  Pose3D T_end_to_begin;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, z, delta orientation
  Eigen::Matrix<double, 6, 6> information;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::map<int, Pose3D, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Pose3D>>>
    MapOfPoses;

typedef std::vector<Constraint, Eigen::aligned_allocator<Constraint>>
    VectorOfConstraints;

bool ReadVertex(std::ifstream* in_file, MapOfPoses* poses) {
  int id;
  Pose3D pose;
  double tx, ty, tz, qw, qx, qy, qz;
  *in_file >> id >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
  Eigen::Vector3d inv_tvec(tx, ty, tz);
  Eigen::Vector4d inv_qvec(qw, qx, qy, qz);

  NormalizeQuaternion(inv_qvec);
  InvertPose(inv_qvec, inv_tvec, &pose.qvec, &pose.tvec);

  // Ensure we don't have duplicate poses.
  if (poses->find(id) != poses->end()) {
    std::cerr << "ERROR: Duplicate vertex with ID: " << id << std::endl;
    return false;
  }
  (*poses)[id] = pose;
  return true;
}

void ReadConstraint(std::ifstream* in_file, VectorOfConstraints* constraints) {
  Constraint constraint;
  *in_file >> constraint.id_begin >> constraint.id_end;

  double tx, ty, tz, qw, qx, qy, qz;
  *in_file >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
  Eigen::Vector3d tvec(tx, ty, tz);
  Eigen::Vector4d qvec(qw, qx, qy, qz);
  NormalizeQuaternion(qvec);

  //Eigen::Vector4d inv_qvec;
  //Eigen::Vector3d inv_tvec;
  //InvertPose(qvec, tvec, &inv_qvec, &inv_tvec);

  constraint.T_end_to_begin.qvec = qvec;
  constraint.T_end_to_begin.tvec = tvec;

  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      *in_file >> constraint.information(i, j);
      if (i != j) {
        constraint.information(j, i) = constraint.information(i, j);
      }
    }
  }

  constraints->push_back(constraint);
}

bool ReadG2OFile(const std::string& filename, MapOfPoses* poses,
                 VectorOfConstraints* constraints) {
  CHECK(poses != nullptr);
  CHECK(constraints != nullptr);

  poses->clear();
  constraints->clear();

  std::ifstream in_file(filename);
  if (!in_file.is_open()) {
    std::cerr << "ERROR: Can't open G2O file: " << filename << std::endl;
    return false;
  }

  std::string data_type;
  while (in_file.good()) {
    in_file >> data_type;
    if (data_type == "VERTEX_SE3:QUAT") {
      if (!ReadVertex(&in_file, poses)) {
        return false;
      }
    } else if (data_type == "EDGE_SE3:QUAT") {
      ReadConstraint(&in_file, constraints);
    } else {
      std::cerr << "ERROR: Unknown data type: " << data_type << std::endl;
      return false;
    }
    // Clear any trailing whitespace from the line.
    in_file >> std::ws;
  }
  return true;
}

bool OutputPoses(const std::string& filename, const MapOfPoses& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    std::cerr << "ERROR: opening the file: " << filename << std::endl;
    return false;
  }
  for (std::map<int, Pose3D, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3D>>>::
           const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose3D, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const int, Pose3D>>>::
        value_type& pair = *poses_iter;
    Eigen::Vector4d inv_qvec;
    Eigen::Vector3d inv_tvec;

    InvertPose(pair.second.qvec, pair.second.tvec, &inv_qvec, &inv_tvec);
    outfile << pair.first << " " << inv_tvec.transpose() << " " << inv_qvec(1)
            << " " << inv_qvec(2) << " " << inv_qvec(3) << " " << inv_qvec(0)
            << '\n';
  }
  return true;
}

void BuildPoseGraphOptimizationProblem(const VectorOfConstraints& constraints,
                                       MapOfPoses* poses,
                                       ceres::Problem* problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);

  if (constraints.empty()) {
    std::cout << "No constraints, no problem to optimize" << std::endl;
    return;
  }

  ceres::LossFunction* loss_function = nullptr;
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;

  for (VectorOfConstraints::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); constraints_iter++) {
    const Constraint& constraint = *constraints_iter;

    MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end());
    MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end());

    std::cout << "edge information matrix: \n" << constraint.information << std::endl;
    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();

    ceres::CostFunction* cost_function = RelativePose6DoFCostFunction::Create(
        constraint.T_end_to_begin.qvec, constraint.T_end_to_begin.tvec,
        sqrt_information);

    problem->AddResidualBlock(
        cost_function, loss_function, pose_end_iter->second.qvec.data(),
        pose_end_iter->second.tvec.data(), pose_begin_iter->second.qvec.data(),
        pose_begin_iter->second.tvec.data());

    problem->SetParameterization(pose_begin_iter->second.qvec.data(),
                                 quaternion_parameterization);
    problem->SetParameterization(pose_end_iter->second.qvec.data(),
                                 quaternion_parameterization);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  MapOfPoses::iterator pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.tvec.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.qvec.data());
}

bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

int main(int argc, char* argv[]) {
  std::cout << "Hello World" << std::endl;

  const std::string g2o_file = "/home/mshe/sphere.g2o";

  MapOfPoses poses;
  VectorOfConstraints constraints;

  CHECK(ReadG2OFile(g2o_file, &poses, &constraints))
      << "Error reading the file: " << g2o_file;

  std::cout << "Number of poses: " << poses.size() << std::endl;
  std::cout << "Number of constraints: " << constraints.size() << std::endl;

  CHECK(OutputPoses("/home/mshe/poses_original.txt", poses));

  ceres::Problem problem;
  BuildPoseGraphOptimizationProblem(constraints, &poses, &problem);

  CHECK(SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(OutputPoses("/home/mshe/poses_optimized.txt", poses));
}