#ifndef SATURATE_HPP
#define SATURATE_HPP

#include <cmath>

inline double saturate( double val, double maxAbsVal )
{
  if( fabs(val) < maxAbsVal )
    return val;

  if( val > maxAbsVal )
    return maxAbsVal;

  if( val < -maxAbsVal )
    return -maxAbsVal;
}

#endif // SATURATE_HPP
