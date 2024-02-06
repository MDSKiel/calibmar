#include "colmap/geometry/pose_prior.h"

#include "colmap/geometry/covariance_transform.h"
#include "colmap/geometry/gps.h"
#include "colmap/geometry/pose.h"
#include "colmap/util/misc.h"

namespace colmap {
namespace {
static const double kNaN = std::numeric_limits<double>::quiet_NaN();
}

PosePrior::PosePrior() : lat0_(kNaN), lon0_(kNaN), depth0_(kNaN) {}

bool PosePrior::Read(const std::string& path,
                     Rigid3d* prior_from_world,
                     Eigen::Matrix7d* prior_from_world_cov) {
  if (!ExistsFile(path)) {
    return false;
  }

  std::ifstream file(path);
  std::string csv_header, csv;
  std::getline(file, csv_header);

  auto csv_header_values = CSVToVector<std::string>(csv_header);

  // For old Girona 500 datasets like e.g. Anton131 and Anton132, the header
  // names in the csv files are different. Therefore use this code below:
  auto iter_lon = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "longitude"));
  auto iter_lat = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "latitude"));
  auto iter_depth = std::distance(
      csv_header_values.begin(),
      std::find(csv_header_values.begin(), csv_header_values.end(),
      "depth"));
  auto iter_yaw = std::distance(
      csv_header_values.begin(),
      std::find(csv_header_values.begin(), csv_header_values.end(), "yaw"));
  auto iter_pitch = std::distance(
      csv_header_values.begin(),
      std::find(csv_header_values.begin(), csv_header_values.end(),
      "pitch"));
  auto iter_roll = std::distance(
      csv_header_values.begin(),
      std::find(csv_header_values.begin(), csv_header_values.end(), "roll"));

  // For newer Girona 500 datasets, like e.g. Anton248, Anton250, Luise258. Use
  // this code below:
  // auto iter_lon = std::distance(csv_header_values.begin(),
  //                               std::find(csv_header_values.begin(),
  //                                         csv_header_values.end(),
  //                                         "Longitude [deg]"));
  // auto iter_lat = std::distance(csv_header_values.begin(),
  //                               std::find(csv_header_values.begin(),
  //                                         csv_header_values.end(),
  //                                         "Latitude [deg]"));
  // // In the AUV-mapping scenario, we extract depth instead of altitude since
  // // the altitude is a relative measure of distance from the vehicle body to the
  // // object's surface.
  // auto iter_depth = std::distance(
  //     csv_header_values.begin(),
  //     std::find(
  //         csv_header_values.begin(), csv_header_values.end(), "Depth [m]"));
  // auto iter_yaw = std::distance(
  //     csv_header_values.begin(),
  //     std::find(
  //         csv_header_values.begin(), csv_header_values.end(), "Yaw [rad]"));
  // auto iter_pitch = std::distance(
  //     csv_header_values.begin(),
  //     std::find(
  //         csv_header_values.begin(), csv_header_values.end(), "Pitch [rad]"));
  // auto iter_roll = std::distance(
  //     csv_header_values.begin(),
  //     std::find(
  //         csv_header_values.begin(), csv_header_values.end(), "Roll [rad]"));

  // Particularly, GEOMAR's robots measure position covariance in NED coordinate
  // system.
  auto iter_north_std = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "North SD [m]"));
  auto iter_east_std = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "East SD [m]"));
  auto iter_depth_std = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "Depth SD [m]"));

  auto iter_yaw_std = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "Yaw SD [rad]"));
  auto iter_pitch_std = std::distance(csv_header_values.begin(),
                                      std::find(csv_header_values.begin(),
                                                csv_header_values.end(),
                                                "Pitch SD [rad]"));
  auto iter_roll_std = std::distance(
      csv_header_values.begin(),
      std::find(
          csv_header_values.begin(), csv_header_values.end(), "Roll SD [rad]"));
  std::getline(file, csv);

  auto csv_values = CSVToVector<std::string>(csv);

  double lat = std::stold(csv_values.at(iter_lat).c_str());
  double lon = std::stold(csv_values.at(iter_lon).c_str());
  double depth = -std::stold(csv_values.at(iter_depth).c_str());
  double yaw = std::stold(csv_values.at(iter_yaw).c_str());
  double pitch = std::stold(csv_values.at(iter_pitch).c_str());
  double roll = std::stold(csv_values.at(iter_roll).c_str());

  // World from prior.
  Rigid3d world_from_prior;

  if (std::isnan(lat0_) && std::isnan(lon0_) && std::isnan(depth0_)) {
    lat0_ = lat;
    lon0_ = lon;
    depth0_ = depth;
  }

  // Converting GPS coordinate system to NED coordinate system.
  GPSTransform gps_transform;

  std::vector<Eigen::Vector3d> ells(1);
  ells[0](0) = lat;
  ells[0](1) = lon;
  ells[0](2) = depth;

  const auto ned = gps_transform.EllToNED(ells, lat0_, lon0_, depth0_)[0];
  world_from_prior.translation.x() = ned(0);
  world_from_prior.translation.y() = ned(1);
  world_from_prior.translation.z() = ned(2);

  // Read yaw, pitch, roll and compute the rotation:
  // yaw: rotation around Z-axis
  // pitch: rotation around Y-axis
  // roll: rotation around X-axis
  // Rotation matrix is computed as: R = Rz * Ry * Rx ("ZYX" order)
  // Note: This rotation matrix rotates a point in the prior coordinate
  // system to the world coordinate system

  world_from_prior.rotation =
      Eigen::Quaterniond(EulerAnglesToRotationMatrix(roll, pitch, yaw));
  *prior_from_world = Inverse(world_from_prior);

  prior_from_world_cov->setZero();

  // Now read covariances if available.
  bool read_cov = true;
  if (iter_north_std == iter_east_std || iter_north_std == iter_depth_std ||
      iter_north_std == iter_yaw_std || iter_north_std == iter_roll_std ||
      iter_north_std == iter_pitch_std) {
    read_cov = false;
  }

  if (read_cov) {
    double n_std = std::stold(csv_values.at(iter_north_std).c_str());
    double e_std = std::stold(csv_values.at(iter_east_std).c_str());
    double d_std = std::stold(csv_values.at(iter_depth_std).c_str());
    double yaw_std = std::stold(csv_values.at(iter_yaw_std).c_str());
    double pitch_std = std::stold(csv_values.at(iter_pitch_std).c_str());
    double roll_std = std::stold(csv_values.at(iter_roll_std).c_str());

    // Positional covariance.
    Eigen::Matrix3d trans_world_from_prior_cov = Eigen::Matrix3d::Identity();
    trans_world_from_prior_cov(0, 0) = std::pow(n_std, 2);
    trans_world_from_prior_cov(1, 1) = std::pow(e_std, 2);
    trans_world_from_prior_cov(2, 2) = std::pow(d_std, 2);

    // Rotational covariance.
    Eigen::Vector3d euler(roll, pitch, yaw);
    Eigen::Matrix3d euler_cov = Eigen::Matrix3d::Identity();
    euler_cov(0, 0) = std::pow(roll_std, 2);
    euler_cov(1, 1) = std::pow(pitch_std, 2);
    euler_cov(2, 2) = std::pow(yaw_std, 2);

    Eigen::Matrix4d rot_world_from_prior_cov;

    CovEulerZYXToQuaternion cov_tform_euler2quat;
    if (!cov_tform_euler2quat.Transform(euler,
                                        euler_cov,
                                        world_from_prior.rotation,
                                        rot_world_from_prior_cov)) {
      prior_from_world_cov->setZero();

    } else {
      Eigen::Matrix7d world_from_prior_cov = Eigen::Matrix7d::Zero();
      world_from_prior_cov.block<4, 4>(0, 0) = rot_world_from_prior_cov;
      world_from_prior_cov.block<3, 3>(4, 4) = trans_world_from_prior_cov;

      // Now invert `world_from_prior` to `prior_from_world`.

      CovRigid3dInverse cov_tform_invert_rigid3d;
      Eigen::Vector4d qvec_dst;
      Eigen::Vector3d tvec_dst;
      Rigid3d tform_dst;
      Eigen::Matrix7d cov_dst;
      if (!cov_tform_invert_rigid3d.Transform(
              world_from_prior, world_from_prior_cov, tform_dst, cov_dst)) {
        prior_from_world_cov->setZero();
      } else {
        *prior_from_world_cov = cov_dst;
      }
    }
  }

  file.close();
  return true;
}
}  // namespace colmap