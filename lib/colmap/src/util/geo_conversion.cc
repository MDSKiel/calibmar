#include "base/pose.h"

#include "util/geo_conversion.h"
#include "util/math.h"

namespace colmap {

GeoConversion::GeoConversion(int utm_zone, bool is_north) {
  OGRErr ogrerror = WGS84_.SetWellKnownGeogCS("WGS84");
  CHECK(ogrerror == OGRERR_NONE);

  // earth center = (0,0,0)
  ogrerror = Cartesian_.SetFromUserInput("EPSG:4978");
  if (ogrerror != OGRERR_NONE) {
    std::cerr << "ERROR: cartesian code not recognized by gdal, error code is "
              << ogrerror << std::endl;
  }

  CHECK(ogrerror == OGRERR_NONE);
  ogrerror = UTM_.SetWellKnownGeogCS("WGS84");
  CHECK(ogrerror == OGRERR_NONE);

  // set up gdal transformations and cache them
  pGPS_to_Cartesian_ = OGRCreateCoordinateTransformation(&WGS84_, &Cartesian_);
  CHECK(pGPS_to_Cartesian_ != nullptr);
  pCartesian_to_GPS_ = OGRCreateCoordinateTransformation(&Cartesian_, &WGS84_);
  CHECK(pCartesian_to_GPS_ != nullptr);

  // set up gdal utm transforms, if utm zone was specified
  pUTM_to_GPS_ = nullptr;
  pGPS_to_UTM_ = nullptr;
  // cache transformation for this utm zone
  SetUTMZone(utm_zone, is_north);
}

GeoConversion::~GeoConversion() {
  if (pUTM_to_GPS_ != nullptr) delete pUTM_to_GPS_;
  if (pGPS_to_UTM_ != nullptr) delete pGPS_to_UTM_;
  delete pGPS_to_Cartesian_;
  delete pCartesian_to_GPS_;
}

void GeoConversion::ComputeUTMTransformations() {
  // reset old utm, if necessary
  if (pGPS_to_UTM_ != nullptr) {
    delete pGPS_to_UTM_;
    pGPS_to_UTM_ = nullptr;
  }
  if (pUTM_to_GPS_ != nullptr) {
    delete pUTM_to_GPS_;
    pUTM_to_GPS_ = nullptr;
  }

  if (utm_zone_ < 0) return;

  // this is the utm object from gdal
  OGRErr ogrerror = UTM_.SetUTM(utm_zone_, is_north_);
  if (ogrerror != OGRERR_NONE) {
    std::cerr << "ERROR: error setting utm zone..." << std::endl;
  }
  CHECK(ogrerror == OGRERR_NONE);

  pGPS_to_UTM_ = OGRCreateCoordinateTransformation(&WGS84_, &UTM_);
  CHECK(pGPS_to_UTM_ != nullptr);

  pUTM_to_GPS_ = OGRCreateCoordinateTransformation(&UTM_, &WGS84_);
  CHECK(pUTM_to_GPS_ != nullptr);
}

void GeoConversion::SetUTMZone(int zone, bool is_north) {
  // OMVASSERT(zone <= 60);
  utm_zone_ = zone;
  is_north_ = is_north;
  ComputeUTMTransformations();
}

double GeoConversion::ApproximateGeodesicDistanceOnSphere(
    const double& latitude1, const double& longitude1, const double& latitude2,
    const double& longitude2) const {
  // compute direction vectors of points on ellpsoid
  Eigen::Vector3d p1 = WGS84ToCartesian(latitude1, longitude1, 0);
  Eigen::Vector3d p2 = WGS84ToCartesian(latitude2, longitude2, 0);
  p1.normalize();
  p2.normalize();
  // compute angle through scalar product of normalized direction vectors
  double angle = acos(p1.dot(p2));
  // full circumference is 2*pi*r, but we only have "angle" instead of 2pi
  // average earth radius 6371 km is used, result is in m
  return angle * AVERAGE_EARTH_RADIUS_M;
}

Eigen::Vector3d GeoConversion::WGS84ToCartesian(const double& latitude_degree,
                                                const double& longitude_degree,
                                                const double& height) const {
  // gdal wants degrees, but swapped order !
  Eigen::Vector3d wgs84(longitude_degree, latitude_degree, height);

  if (!pGPS_to_Cartesian_->Transform(1, &(wgs84[0]), &(wgs84[1]),
                                     &(wgs84[2]))) {
    std::cerr << "ERROR: coordinate transform failed" << std::endl;
    wgs84 = Eigen::Vector3d::Zero();
  }
  return wgs84;
}

Eigen::Vector3d GeoConversion::CartesianToWGS84(
    const Eigen::Vector3d& euc) const {
  Eigen::Vector3d wgs84(euc);

  if (!pCartesian_to_GPS_->Transform(1, &(wgs84[0]), &(wgs84[1]),
                                     &(wgs84[2]))) {
    std::cerr << "ERROR: coordinate transform failed" << std::endl;
    wgs84 = Eigen::Vector3d::Zero();
  }
  // gdal returns degrees, but swapped order !
  double swapbuffer = wgs84[0];
  wgs84[0] = wgs84[1];
  wgs84[1] = swapbuffer;

  return wgs84;
}

int GeoConversion::GetCartesianNorthAndGravityVectors(
    const double& latitude_degree, const double& longitude_degree,
    Eigen::Vector3d& north, Eigen::Vector3d& gravity) const {
  int returnvalue = 0;
  gravity = WGS84ToCartesian(latitude_degree, longitude_degree);
  Eigen::Vector3d northpole(WGS84ToCartesian(90.0, 0.0));
  Eigen::Vector3d northvector(northpole - gravity);
  gravity.normalize();
  // remove components in direction of gravity from northvector
  if (latitude_degree < 0.0) {
    // southern hemisphere
    gravity *= -1.0;
    north = northvector - (gravity * gravity.transpose()) * northvector;
  } else {
    // northern hemisphere
    north = northvector - (gravity * gravity.transpose()) * northvector;
    gravity *= -1.0;
  }

  if (north.norm() < 1e-5) {
    returnvalue = 1;  // we are at the north pole !
    north = Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  north.normalize();
  // make sure north is orthogonal to gravity !
  CHECK(fabs(north.dot(gravity)) < 1e-5);

  return returnvalue;
}

int GeoConversion::GetCartesianOrientation(const double& latitude_degree,
                                           const double& longitude_degree,
                                           const double& yaw_degree,
                                           const double& pitch_degree,
                                           const double& roll_degree,
                                           Eigen::Matrix3d& orientation) const {
  Eigen::Vector3d north, gravity;
  int returnvalue = GetCartesianNorthAndGravityVectors(
      latitude_degree, longitude_degree, north, gravity);
  if (returnvalue != 0) {
    std::cerr << "ERROR: error retrieving directions fromn lat/lon"
              << std::endl;
    return returnvalue;
  }

  Eigen::Vector3d east = gravity.cross(north);
  /*
  RMatrix RYaw(MatrixIdentity), RPitch(MatrixIdentity), RRoll(MatrixIdentity);
  RPitch[2][2] = RPitch[1][1] = cos(nd.pitch_degree/180.0*M_PI);
  RPitch[1][2] = -sin(pitch_degree/180.0*M_PI);
  RPitch[2][1] = -RPitch[1][2];

  RYaw[1][1] = RYaw[0][0] = cos(yaw_degree/180.0*M_PI);
  RYaw[0][1] = sin(yaw_degree/180.0*M_PI);
  RYaw[1][0] = -RYaw[0][1];


  // roll is not yet supported
  OMVASSERT(roll_degree == 1.0);
  */

  // nd.R.SetZYX(nd.pitch/180.0*M_PI, nd.heading/180.0*M_PI,
  // nd.roll/180.0*M_PI);

  // according to wikipedia as of 2014-06-03
  Eigen::Matrix3d earthtangentframe_to_vehicle;
  Eigen::Matrix3d Rx =
      Eigen::AngleAxisd(roll_degree / -180.0 * M_PI, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d Ry =
      Eigen::AngleAxisd(pitch_degree / -180.0 * M_PI, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  Eigen::Matrix3d Rz =
      Eigen::AngleAxisd(yaw_degree / -180.0 * M_PI, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  earthtangentframe_to_vehicle = Rx * Ry * Rz;

  // this matrix can convert the gravity vector 0,0,9.81 defined in the the
  // Earth's local tangent plane into ROV-based coordinates (x=forward,
  // y=starboard/"right", z=ship floor/"down")

  Eigen::Matrix3d cartesian_to_tangentframe;
  cartesian_to_tangentframe(0, 0) = north(0);
  cartesian_to_tangentframe(0, 1) = north(1);
  cartesian_to_tangentframe(0, 2) = north(2);
  cartesian_to_tangentframe(1, 0) = east(0);
  cartesian_to_tangentframe(1, 1) = east(1);
  cartesian_to_tangentframe(1, 2) = east(2);
  cartesian_to_tangentframe(2, 0) = gravity(0);
  cartesian_to_tangentframe(2, 1) = gravity(1);
  cartesian_to_tangentframe(2, 2) = gravity(2);

  orientation = earthtangentframe_to_vehicle * cartesian_to_tangentframe;
  // now orientation converts from cartesian earth directions (westafrica,
  // singapore, northpole) coordinates to local vehicle coordinates (front,
  // starboard, bottom)

  return returnvalue;
}

int GeoConversion::GetUTMOrientation(const double& yaw_degree,
                                     const double& pitch_degree,
                                     const double& roll_degree,
                                     Eigen::Matrix3d& orientation) const {
  // TODO: Make this more efficient, no need for matrix multiplications, just
  // swap entries !

  Eigen::Vector3d north(0.0, 1.0, 0.0), gravity(0.0, 0.0, -1.0);
  Eigen::Vector3d east(1.0, 0.0, 0.0);  // = gravity.CrossProduct(north);

  Eigen::Matrix3d earthtangentframe_to_vehicle;

  // this is how it should be according to wikipedia as of 2014-06-03
  Eigen::Matrix3d Rx =
      Eigen::AngleAxisd(roll_degree / -180.0 * M_PI, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  Eigen::Matrix3d Ry =
      Eigen::AngleAxisd(pitch_degree / -180.0 * M_PI, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  Eigen::Matrix3d Rz =
      Eigen::AngleAxisd(yaw_degree / -180.0 * M_PI, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();

  earthtangentframe_to_vehicle = Rx * Ry * Rz;

  // this matrix can convert the gravity vector 0,0,9.81 defined in the the
  // Earth's local tangent plane into ROV-based coordinates (x=forward,
  // y=starboard/"right", z=ship floor/"down")

  Eigen::Matrix3d cartesian_to_tangentframe;
  cartesian_to_tangentframe(0, 0) = north(0);
  cartesian_to_tangentframe(0, 1) = north(1);
  cartesian_to_tangentframe(0, 2) = north(2);
  cartesian_to_tangentframe(1, 0) = east(0);
  cartesian_to_tangentframe(1, 1) = east(1);
  cartesian_to_tangentframe(1, 2) = east(2);
  cartesian_to_tangentframe(2, 0) = gravity(0);
  cartesian_to_tangentframe(2, 1) = gravity(1);
  cartesian_to_tangentframe(2, 2) = gravity(2);

  orientation = earthtangentframe_to_vehicle * cartesian_to_tangentframe;
  // now orientation converts from UTM directions (east, north, up) coordinates
  // to local vehicle coordinates (front, starboard, bottom)
  return 0;
}

int GeoConversion::GetAnglesFromUTMOrientation(const Eigen::Matrix3d& R,
                                               double& heading_degree,
                                               double& pitch_degree,
                                               double& roll_degree) const {
  // TODO: Make this more efficient, no need for matrix multiplications, just
  // swap entries !

  Eigen::Vector3d north(0.0, 1.0, 0.0), gravity(0.0, 0.0, -1.0);
  Eigen::Vector3d east(1.0, 0.0, 0.0);  // = gravity.CrossProduct(north);
  Eigen::Matrix3d cartesian_to_tangentframe;
  cartesian_to_tangentframe(0, 0) = north(0);
  cartesian_to_tangentframe(0, 1) = north(1);
  cartesian_to_tangentframe(0, 2) = north(2);
  cartesian_to_tangentframe(1, 0) = east(0);
  cartesian_to_tangentframe(1, 1) = east(1);
  cartesian_to_tangentframe(1, 2) = east(2);
  cartesian_to_tangentframe(2, 0) = gravity(0);
  cartesian_to_tangentframe(2, 1) = gravity(1);
  cartesian_to_tangentframe(2, 2) = gravity(2);

  Eigen::Matrix3d earthtangentframe_to_vehicle =
      R * cartesian_to_tangentframe.transpose();
  RotationMatrixToEulerAngles(earthtangentframe_to_vehicle, &roll_degree,
                              &pitch_degree, &heading_degree);
  roll_degree *= -1.0;
  pitch_degree *= -1.0;
  heading_degree *= -1.0;
  return 0;
}

double GeoConversion::GetDepth(const Eigen::Vector3d& cartesian) const {
  Eigen::Vector3d wgs84 = this->CartesianToWGS84(cartesian);
  wgs84(2) = 0;
  Eigen::Vector3d cartesiansurface = this->WGS84ToCartesian(wgs84(0), wgs84(1));
  return (cartesiansurface - cartesian).norm();
}

/** @brief convert from UTM to GPS, must set UTM zone before ! */
Eigen::Vector3d GeoConversion::UTMToWGS84(const double& easting,
                                          const double& northing,
                                          const double& height) const {
  // must know the utm zone, "-1" means uninitialized
  CHECK(utm_zone_ >= 0);

  Eigen::Vector3d wgs84(easting, northing, height);
  // transform 1 element at the specified address
  if (!pUTM_to_GPS_->Transform(1, &(wgs84(0)), &(wgs84(1)), &(wgs84(2)))) {
    std::cerr << "ERROR: coordinate transform failed" << std::endl;
    wgs84 = Eigen::Vector3d(0.0, 0.0, 0.0);
  }
  // gdal returns degrees, but swapped order !
  double swapbuffer = wgs84(0);
  wgs84(0) = wgs84(1);
  wgs84(1) = swapbuffer;

  return wgs84;
}

Eigen::Vector3d GeoConversion::WGS84ToUTM(const double& latitude_degree,
                                          const double& longitude_degree,
                                          const double& height) {
  if (utm_zone_ < 0) {
    SetUTMZone(ComputeUTMZone(longitude_degree), latitude_degree > 0.0);
    std::cout << "WARN: No utm zone specified, auto-selected as " << utm_zone_
              << ((latitude_degree > 0.0) ? "north" : "south") << std::endl;
  }
  // gdal wants degrees, but swapped order !
  Eigen::Vector3d wgs84(longitude_degree, latitude_degree, height);

  if (!pGPS_to_UTM_->Transform(1, &(wgs84[0]), &(wgs84[1]), &(wgs84[2]))) {
    std::cerr << "ERROR: coordinate transform failed " << wgs84 << std::endl;
    wgs84 = Eigen::Vector3d(0.0, 0.0, 0.0);
  }
  return wgs84;
}

double GeoConversion::OneMeterToDegreesLongitude(
    const double& latitude_degrees, const double& longitude_degree) {
  CHECK(GetUTMZone() != -1)
      << "UTM zone is unspecified, needed for meter to degree "
         "calculation";

  // Three-vectors for GPS/WGS84 are encoded (latitude, longitude,
  // height-above-ellipsoid), three-vectors for UTM are (somewhat
  // inconsistently) encoded as (easting, northing, height-ab.-ell.),
  double center_merian = 6.0 * (double(GetUTMZone()) - 1.5) - 180.0;
  if (longitude_degree != -1.0) center_merian = longitude_degree;
  Eigen::Vector3d east_north_height =
      WGS84ToUTM(latitude_degrees, center_merian);
  Eigen::Vector3d lat_lon_height = UTMToWGS84(
      east_north_height(0), east_north_height(1), east_north_height(2));
  return (UTMToWGS84(east_north_height(0) + 1.0, east_north_height(1),
                     east_north_height(2))(1) -
          lat_lon_height(1));
}

double GeoConversion::OneMeterToDegreesLatitude(
    const double& latitude_degrees, const double& longitude_degree) {
  CHECK(GetUTMZone() != -1)
      << "UTM zone is unspecified, needed for meter to degree calculation";
  // Three-vectors for GPS/WGS84 are encoded (latitude, longitude,
  // height-above-ellipsoid), three-vectors for UTM are (somewhat
  // inconsistently) encoded as (easting, northing, height-ab.-ell.),
  double center_merian = 6.0 * (double(GetUTMZone()) - 1.5) - 180.0;
  // cout<<"center meridian for zone "<<GetUTMZone()<<" computed as "<<center
  // merian<<endl;
  if (longitude_degree != -1.0) center_merian = longitude_degree;
  Eigen::Vector3d east_north_height =
      WGS84ToUTM(latitude_degrees, center_merian);
  // cout<<"east_north_height is "<<east_north_height<<endl;
  Eigen::Vector3d lat_lon_height = UTMToWGS84(
      east_north_height(0), east_north_height(1), east_north_height(2));
  // cout<<"lat_lon_height is "<<lat_lon_height<<endl;
  return (UTMToWGS84(east_north_height(0), east_north_height(1) + 1.0,
                     east_north_height(2))(0) -
          lat_lon_height(0));
}

double GeoConversion::OneMinuteLongitudeToMeters(
    const double& latitude_degrees, const double& longitude_degree) {
  CHECK(GetUTMZone() != -1)
      << "UTM zone is unspecified, needed for meter to degree calculation";
  // Three-vectors for GPS/WGS84 are encoded (latitude, longitude,
  // height-above-ellipsoid), three-vectors for UTM are (somewhat
  // inconsistently) encoded as (easting, northing, height-ab.-ell.),
  double center_merian = 6.0 * (double(GetUTMZone()) - 1.5) - 180.0;
  if (longitude_degree != -1.0) center_merian = longitude_degree;
  Eigen::Vector3d east_north_height =
      WGS84ToUTM(latitude_degrees, center_merian);
  return WGS84ToUTM(latitude_degrees, center_merian + 1.0 / 60.0)(0) -
         east_north_height(0);
}

double GeoConversion::OneMinuteLatitudeToMeters(
    const double& latitude_degrees, const double& longitude_degree) {
  CHECK(GetUTMZone() != -1)
      << "UTM zone is unspecified, needed for meter to degree calculation";
  // Three-vectors for GPS/WGS84 are encoded (latitude, longitude,
  // height-above-ellipsoid), three-vectors for UTM are (somewhat
  // inconsistently) encoded as (easting, northing, height-ab.-ell.),
  double center_merian = 6.0 * (double(GetUTMZone()) - 1.5) - 180.0;
  if (longitude_degree != -1.0) center_merian = longitude_degree;
  Eigen::Vector3d east_north_height =
      WGS84ToUTM(latitude_degrees, center_merian);
  return WGS84ToUTM(latitude_degrees + 1.0 / 60.0, center_merian)(1) -
         east_north_height(1);
}

int GeoConversion::ParseUTMZone(const std::string& utm_zone_string,
                                int& utm_zone, bool& north) {
  if (utm_zone_string.length() == 0 || utm_zone_string.length() > 3) {
    std::cerr << "ERROR: could not parse utm zone from string "
              << utm_zone_string << std::endl;
    return -1;
  }
  if (utm_zone_string.length() == 3) {
    if (utm_zone_string[0] < '0' || utm_zone_string[0] > '9') {
      std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                << std::endl;
      return -1;
    }
    if (utm_zone_string[1] < '0' || utm_zone_string[1] > '9') {
      std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                << std::endl;
      return -1;
    }
    utm_zone = 10 * (utm_zone_string[0] - '0') + utm_zone_string[1] - '0';
    if (utm_zone_string[2] == 'S' || utm_zone_string[2] == 's') north = false;
  } else if (utm_zone_string.length() == 2) {
    if (utm_zone_string[1] == 'S') {
      if (utm_zone_string[0] < '0' || utm_zone_string[0] > '9') {
        std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                  << std::endl;
        return -1;
      }
      utm_zone = utm_zone_string[0] - '0';
      north = false;
    } else if (utm_zone_string[1] == 'N' || utm_zone_string[1] == 'n') {
      if (utm_zone_string[0] < '0' || utm_zone_string[0] > '9') {
        std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                  << std::endl;
        return -1;
      }
      utm_zone = utm_zone_string[0] - '0';
      north = true;
    } else {
      if (utm_zone_string[0] < '0' || utm_zone_string[0] > '9') {
        std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                  << std::endl;
        return -1;
      }
      if (utm_zone_string[1] < '0' || utm_zone_string[1] > '9') {
        std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                  << std::endl;
        return -1;
      }
      utm_zone = 10 * (utm_zone_string[0] - '0') + utm_zone_string[1] - '0';
    }
  } else if (utm_zone_string.length() == 1) {
    if (utm_zone_string[0] < '0' || utm_zone_string[0] > '9') {
      std::cerr << "ERROR: parse error in utm zone: " << utm_zone_string
                << std::endl;
      return -1;
    }
    utm_zone = utm_zone_string[0] - '0';
  }
  return 0;
}
}  // namespace colmap