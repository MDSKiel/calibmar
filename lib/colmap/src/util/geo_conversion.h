#ifndef COLMAP_SRC_UTIL_GEO_CONVERSION_H_
#define COLMAP_SRC_UTIL_GEO_CONVERSION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <gdal/ogr_spatialref.h>

#include "util/logging.h"

#define AVERAGE_EARTH_RADIUS_M 6371000.0

namespace colmap {

/** @brief class for converting between different coordinate systems, "GPS",
 * UTM, Cartesian GPS/WGS84 latitude is negative for south and longitude is
 * negative for west. For cartesian coordinates the axes are x-axis points to
 * west africa (lat=0, lon=0) y-axis points to singapore (lat=0, lon=90°) z-axis
 * points to north pole (lat=90°, lon=0) Three vectors for cartesian are in
 * meter with respect to the center of the earth. Three-vectors for GPS/WGS84
 * are encoded (latitude, longitude, height-above-ellipsoid), three-vectors for
 * UTM are (somewhat inconsistently) encoded as (easting, northing,
 * height-ab.-ell.),
 *
 *         It seems that GDAL does not incorporate th 10 000 000 northing offset
 * for the southern hemisphere such that currently southern hemisphere northing
 * is negative !
 *
 *  @attention latitude and longitude are specified in degree (not in radians !)
 *  @author kkoeser 05/2014
 * */
class GeoConversion {
 public:
  /** @brief set up OGR/GDAL conversion objects
   * @param utm_zone if you want to do utm conversions, then you have to specify
   * the utm zone before. This is not needed for simple GPS to Cartesian
   * conversions */
  GeoConversion(int utm_zone = -1, bool is_north = true);

  /** @brief delete all conversion objects */
  ~GeoConversion();

  /** @brief return object's utm zone or -1 if this has not been set */
  inline int GetUTMZone() const;

  /** @brief return object's utm zone or -1 if this has not been set */
  inline int IsUTMNorth() const;

  /** @brief set (another) utm zone, only needed when utm coordinates are needed
   * for i/o */
  void SetUTMZone(int zone, bool is_north = true);

  /** @brief compute UTM zone (zones are 6 degrees wide longitude stripes) for
   * longitude in degree */
  inline int ComputeUTMZone(const double& longitude_degree) const;

  /** @brief compute "shortest walking distance" in m from (lat1;lon1) to
   * (lat2;lon2) on assumed spherical earth surface with radius 6371km
   *
   * @attention: this is an approximation without considering actual shape of
   * the earth (earth as sphere): polar radius is 0.335% smaller than equator
   * radius (this mean 20 km!) and so the computed distance comes with like 1%
   * uncertainty.
   */
  double ApproximateGeodesicDistanceOnSphere(const double& latitude1,
                                             const double& longitude1,
                                             const double& latitude2,
                                             const double& longitude2) const;

  /** @brief helper method: convert arc seconds to decimal degrees, e.g. 64° 15'
   * 36" to 64.26° */
  inline double ArcToDecimal(const double& degrees, const double& minutes,
                             const double& seconds = 0.0) const;

  /** @brief convert GPS coordinates into an earth-centric, Cartesian coordinate
   * system ("meters from center") */
  Eigen::Vector3d WGS84ToCartesian(const double& latitude_degree,
                                   const double& longitude_degree,
                                   const double& height = 0.0) const;

  /** @brief convert earth-centric, Cartesian coordinates into
   * latitude/longitude */
  Eigen::Vector3d CartesianToWGS84(const Eigen::Vector3d& euc) const;

  /** @brief for a given GPS position, return north and gravity as a cartesian
   * 3-vector
   *
   * For now, assume earth as sphere, maybe later use appropriate function from
   * GDAL
   * @return 0=ok, >0 warning: north undefined for poles */
  int GetCartesianNorthAndGravityVectors(const double& latitude_degree,
                                         const double& longitude_degree,
                                         Eigen::Vector3d& north,
                                         Eigen::Vector3d& gravity) const;

  /** @brief for a given GPS position and heading/pitch/roll information, return
   * "pose" that transforms a world direction in the Earth frame into a
   * direction in ROV's body frame (x=front, y=right, z=down)
   *
   *  @attention For now, assume earth as sphere, maybe later use appropriate
   * function from GDAL
   *  @return 0=ok */
  int GetCartesianOrientation(const double& latitude_degree,
                              const double& longitude_degree,
                              const double& yaw_degree,
                              const double& pitch_degree,
                              const double& roll_degree,
                              Eigen::Matrix3d& orientation) const;

  /** @brief for given heading, pitch, roll values compute a pose that
   * transforms a world direction in UTM (easting; northing; up)  into a local
   * direction in ROV coordinates
   * @attention angles are in degree
   */
  int GetUTMOrientation(const double& yaw_degree, const double& pitch_degree,
                        const double& roll_degree,
                        Eigen::Matrix3d& orientation) const;

  /** @brief given an RMatrix R that describes the orientation of an ROV in the
     UTM frame, compute its heading, pitch and roll
      @attention angles are in degree */
  int GetAnglesFromUTMOrientation(const Eigen::Matrix3d& R,
                                  double& heading_degree, double& pitch_degree,
                                  double& roll_degree) const;

  /** @brief get depth value (m below sea level) of cartesian 3d point */
  double GetDepth(const Eigen::Vector3d& cartesian) const;

  /** @brief convert from UTM to GPS, must set UTM zone before ! */
  Eigen::Vector3d UTMToWGS84(const double& easting, const double& northing,
                             const double& height = 0.0) const;

  /** @brief convert from GPS to UTM, must set UTM zone before ! */
  Eigen::Vector3d WGS84ToUTM(const double& latitude_degree,
                             const double& longitude_degree,
                             const double& height = 0.0);

  /** @brief return by which number the longitude (at latitude_degrees) changes
     when going one meter east.
      @param longitude_degree optional parameter where you can specify the
     actual longitude where to compute. If -1.0, the center merian of the used
     UTM Zone is used. You can however also specify it, in case you want to
     exploit the special ellipsoidal shape of the earth in the WGS84 model. Note
     that you should use a longitude within the current UTM zone, other wise you
     will get big errors.
      @param latitude_degrees the latitude (in degrees) where you want the
     transformation. One meter means more degrees longitude when closer to the
     poles  */
  double OneMeterToDegreesLongitude(const double& latitude_degrees,
                                    const double& longitude_degree = -1.0);

  /** @brief return by which number the latitude changes when going one meter
     east
      @param longitude_degree optional parameter where you can specify the
     actual longitude where to compute. If -1.0, the center merian of the used
     UTM Zone is used. You can however also specify it, in case you want to
     exploit the special ellipsoidal shape of the earth in the WGS84 model. Note
     that you should use a longitude within the current UTM zone, other wise you
     will get big errors.
      @param latitude_degrees the latitude (in degrees) where you want the
     transformation. Should be almost constant (1 minute is a nautical mile). */
  double OneMeterToDegreesLatitude(const double& latitude_degrees = 5.0,
                                   const double& longitude_degree = -1.0);

  /** @brief return how many meters (UTM) a step of one minute (1/60 degree) is
      @param longitude_degree optional parameter where you can specify the
     actual longitude where to compute. If -1.0, the center merian of the used
     UTM Zone is used. You can however also specify it, in case you want to
     exploit the special ellipsoidal shape of the earth in the WGS84 model. Note
     that you should use a longitude within the current UTM zone, other wise you
     will get big errors.
      @param latitude_degrees the latitude (in degrees) where you want the
     transformation. One meter means more degrees longitude when closer to the
     poles  */
  double OneMinuteLongitudeToMeters(const double& latitude_degrees,
                                    const double& longitude_degree = -1.0);

  /** @brief return how many meters (UTM) a step of one minute (1/60 degree) is
      @param longitude_degree optional parameter where you can specify the
     actual longitude where to compute. If -1.0, the center merian of the used
     UTM Zone is used. You can however also specify it, in case you want to
     exploit the special ellipsoidal shape of the earth in the WGS84 model. Note
     that you should use a longitude within the current UTM zone, other wise you
     will get big errors.
      @param latitude_degrees the latitude (in degrees) where you want the
     transformation. Should be almost constant (1 minute is a nautical mile). */
  double OneMinuteLatitudeToMeters(const double& latitude_degrees = 5.0,
                                   const double& longitude_degree = -1.0);

  /** @brief parses the string in utmzonestring, e.g. "16S", and returns zone in
   * utmzone and true in north (if north) or false (south)
   *  @return 0 on success, other value indicate parse error
   *  @param utm_zone_string [d]d[S|N] where d is a digit and S and N are capital
   * letter. If letter is ommitted, north is assumed */
  static int ParseUTMZone(const std::string& utm_zone_string, int& utm_zone,
                          bool& north);

 protected:
  // Called after utm zone has changed to update mappings
  void ComputeUTMTransformations();

  // Transformation objects for different transforms from GDAL.
  OGRCoordinateTransformation* pGPS_to_Cartesian_ = nullptr;
  OGRCoordinateTransformation* pCartesian_to_GPS_ = nullptr;
  OGRCoordinateTransformation* pGPS_to_UTM_ = nullptr;
  OGRCoordinateTransformation* pUTM_to_GPS_ = nullptr;

  // Reference systems from GDAL.
  OGRSpatialReference UTM_, WGS84_, Cartesian_;

  // Backup utm zone used in transformation objects.
  int utm_zone_;

  // Needed for UTM equator problem.
  bool is_north_;

};  // class

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

double GeoConversion::ArcToDecimal(const double& degrees, const double& minutes,
                                   const double& seconds) const {
  return ((seconds / 60.0 + minutes) / 60.0) + degrees;
}

int GeoConversion::GetUTMZone() const { return utm_zone_; }

int GeoConversion::IsUTMNorth() const { return is_north_; }

int GeoConversion::ComputeUTMZone(const double& longitude_degree) const {
  return int((longitude_degree + 180.0) / 6.0) + 1;
}
}  // namespace colmap

#endif  // COLMAP_SRC_UTIL_ENDIAN_H