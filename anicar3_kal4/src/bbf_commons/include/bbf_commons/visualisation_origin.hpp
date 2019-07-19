// File:           visualisation_origin.hpp
// Creation Date:  Thursday, March 17 2011
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(VISUALISATION_ORIGIN_HPP)
#define VISUALISATION_ORIGIN_HPP

#include <bbf_commons/convert_coordinates.hpp>

// Offset everything (computationally) by this lat lon pair.
// This is necessary because GPUs internally work with single
// precision, which becomes an issue for large distances (Millions of meters)
// from the origin

const double visualisation_origin_lat = 49.;
const double visualisation_origin_lon = 8.4;

// only use if LatLonFrame won't do, for example for map like structures, which
// must be  transformed on a "per vertex"-basis
template<class float_type>
inline void lat_lon_to_visualisation( double lat, double lon, float_type& vis_mx, float_type& vis_my ) 
{
  double vis_offs_mx,vis_offs_my;
  convert_coordinates::latlon_to_mercator( lat, lon, 1., vis_mx, vis_my );
  convert_coordinates::latlon_to_mercator( visualisation_origin_lat, visualisation_origin_lon, 1., vis_offs_mx, vis_offs_my );
  vis_mx -= vis_offs_mx; // - 0.45
  vis_my -= vis_offs_my;
};

// undo lat_lon_to_visualisation
template<class float_type>
inline void visualisation_to_lat_lon( float_type vis_mx, float_type vis_my, double &lat, double &lon )
{
  double vis_offs_mx, vis_offs_my;
  convert_coordinates::latlon_to_mercator( visualisation_origin_lat, visualisation_origin_lon, 1., vis_offs_mx, vis_offs_my );
  vis_mx += vis_offs_mx;
  vis_my += vis_offs_my;
  convert_coordinates::mercator_to_latlon (vis_mx, vis_my, 1., lat, lon);
};



#endif
