#ifndef RESERVE_FUNCTION_HPP
#define RESERVE_FUNCTION_HPP

#include <utility>

const double EXTRA_FREESPACE_WHEN_BLOCKED = 1.5;
const double BASE_RESERVE    = 0.45;
const double MINIMUM_RESERVE = 0.35;
const double ABSOLUTE_MINIMUM_RESERVE = 0.30;

const double DYN_OBJ_LENGTH  = 5.0;
const double DYN_OBJ_WIDTH   = 2.5; // 3.0;
const double BICYCLE_RESERVE = 0.6;

// distance dependent reserve for obstacles
inline double reserve( double distance, double base_reserve )
{
  // return 0.0;
  return std::min( 0.3, distance * /*0.02*/ 0.02 ) + base_reserve;
}

inline double longitudinal_stx_reserve( double distance )
{
  // return 0.0;
  return std::min( 2.0, distance * distance * 0.0035 ) + 1.5;
}

#endif // RESERVE_FUNCTION_HPP
