#ifndef LOCALCARTESIANCSFROMGPS_H_
#define LOCALCARTESIANCSFROMGPS_H_

#include "bbf_commons/convert_coordinates.hpp"

/*!
 * \class LocalCartesianCSFromGPS
 * \brief Allows convenient transformation from Oxts data into a local xyz-coordinate frame
 *
 * \todo  include angles
 *
 * \author Frank Moosmann <moosmann@mrt.uka.de>
 **/
class LocalCartesianCSFromGPS
{
public:
  LocalCartesianCSFromGPS() : initialized(false) {};
  template<typename struct_t> // e.g. GCDC::OxtsLite
  LocalCartesianCSFromGPS(const struct_t &insData_) : initialized(false) {initialize(insData_);} ;
  LocalCartesianCSFromGPS(double lat0_, double lon0_, double alt0_) : initialized(false) {initialize(lat0_, lon0_, alt0_);};
  
  void getXZY(double lat, double lon, double alt, double &x, double &y, double &z) {
    initialize(lat, lon, alt);
    convert_coordinates::latlon_diff_to_meters(lat0, lon0, lat, lon, x, y);
    z = alt - alt0;
  };
  
  template<typename struct_t>
  void getXZY(const struct_t &insData, double &x, double &y, double &z) {
    getXZY(insData.lat, insData.lon, insData.alt, x, y, z);
  };
  
private:
  bool initialized;
  double lat0, lon0, alt0;//, head0;
  void initialize(double lat0_, double lon0_, double alt0_) {
    if (!initialized) {
      lat0 = lat0_;
      lon0 = lon0_;
      alt0 = alt0_;
      initialized = true;
    }
  };
  template<typename struct_t> // e.g. GCDC::OxtsLite
  void initialize(const struct_t &insData) {
    initialize(insData.lat, insData.lon, insData.alt);
  };
};

#endif /*LOCALCARTESIANCSFROMGPS_H_*/
