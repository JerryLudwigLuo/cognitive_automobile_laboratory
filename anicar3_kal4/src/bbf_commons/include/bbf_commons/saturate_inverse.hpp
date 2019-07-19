#ifndef SATURATE_INVERSE_HPP
#define SATURATE_INVERSE_HPP

#include <cmath>

inline double saturate_inverse( double val, double minAbsVal )
{
  if( fabs(val) > minAbsVal )
    return val;

  if( val >= 0 )
    return minAbsVal;

  if( val < 0 )
    return -minAbsVal;
}

#endif // SATURATE_INVERSE_HPP
