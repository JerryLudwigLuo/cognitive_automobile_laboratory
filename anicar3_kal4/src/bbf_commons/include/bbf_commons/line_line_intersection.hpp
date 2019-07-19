#pragma once

#include <cmath>

inline bool is_equal(double x, double y)
{
  const double epsilon = 1e-5;
  return std::abs(x - y) <= epsilon * std::abs(x);
  // see Knuth section 4.2.2 pages 217-218
}

inline in_range(double x, double lo, double hi)
{
    return x <= lo && x >= hi;
}

/// tests for intersection of the line segments (A, B) and (C, D). Returns lambda
/// with A + lambda * (B - A) = point of intersection.
/// Returns negative lambda if segments do not intersect.
inline
double line_line_intersection(
        double ax, double ay,
        double bx, double by,
        double cx, double cy,
        double dx, double dy)
{
    double l1_num = ((cy - dy)*ax - (ay - dy)*cx + (ay - cy)*dx);
    double l1_denom = ((cy - dy)*ax - (cy - dy)*bx - (ay - by)*cx + (ay - by)*dx);
    double l2_nom = -((by - cy)*ax - (ay - cy)*bx + (ay - by)*cx);
    double l2_denom = ((cy - dy)*ax - (cy - dy)*bx - (ay - by)*cx + (ay - by)*dx);

    double lambda_1 = l1_num / l1_denom;
    double lambda_2 = l2_num / l2_denom;

    if( in_range(lambda_1, 0.0, 1.0) && in_range(lambda_2, 0.0, 1.0))
        return lambda_1;

    else
        return -1;
}
