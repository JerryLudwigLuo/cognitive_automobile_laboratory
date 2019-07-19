#ifndef LINEAR_INTERPOLATION_HPP
#define LINEAR_INTERPOLATION_HPP

#include "bbf_commons/normalize_angle.hpp"

inline double linear_interpolation( double v0, double v1, double lambda )
{
  return v1*lambda + v0*(1-lambda);;
}

inline double linear_interpolation_angle( double v0, double v1, double lambda )
{
  // normalise one angle from the other
  while( v1 - v0 > M_PI ) v0 += 2*M_PI;
  while( v0 - v1 > M_PI ) v1 += 2*M_PI;

  return normalize_angle( linear_interpolation(v0,v1,lambda) );
}

#endif // LINEAR_INTERPOLATION_HPP
