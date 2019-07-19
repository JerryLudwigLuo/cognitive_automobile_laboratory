// File:           convex_hull.hpp
// Creation Date:  Monday, April 23 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(CONVEX_HULL_HPP)
#define CONVEX_HULL_HPP

#include <vector>

void offset_polyline( const std::vector<double>& xs_in, const std::vector<double>& ys_in, std::vector<double>& xs_out, std::vector<double>& ys_out, double offset = 1. );

void offset_polyline( const double* xs_in, const double* ys_in, double* xs_out, double* ys_out, int N, double offset = 1. );

void filter_max_m( const std::vector<double>& xs_in, const std::vector<double>& ys_in, std::vector<double>& xs_out, std::vector<double>& ys_out, double m_max = 0.5 );

void convex_hull( const std::vector<double>& xs_in, const std::vector<double>& ys_in, std::vector<double>& xs_out, std::vector<double>& ys_out );

#endif
