#ifndef LINE_SEGMENT_INTERSECTION_HPP
#define LINE_SEGMENT_INTERSECTION_HPP

////////////////////////////////////////////////////////////////////////////////
//
// 2D Line Segment Intersection example
// Implementation of the theory provided by Paul Bourke
//
// Written by Damian Coventry
// Tuesday, 9 January 2007
//
////////////////////////////////////////////////////////////////////////////////

/*
  Extension for usage with libgeometry by Philipp Bender <pbender@fzi.de>, September 25, 2012
  */

#include <iostream>
#include "bbf_geometry/geometry.h"
#include "bbf_commons/pairwise_iterator.hpp"

namespace Coventry{ // original author



class Vector
{
public:
    float x_, y_;

    Vector(float f = 0.0f)
        : x_(f), y_(f) {}

    Vector(float x, float y)
        : x_(x), y_(y) {}
};

class LineSegment
{
public:
    Vector begin_;
    Vector end_;

    LineSegment(const Vector& begin, const Vector& end)
        : begin_(begin), end_(end) {}

    enum IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING };

    IntersectResult Intersect(const LineSegment& other_line, Vector& intersection)
    {
        float denom = ((other_line.end_.y_ - other_line.begin_.y_)*(end_.x_ - begin_.x_)) -
                      ((other_line.end_.x_ - other_line.begin_.x_)*(end_.y_ - begin_.y_));

        float nume_a = ((other_line.end_.x_ - other_line.begin_.x_)*(begin_.y_ - other_line.begin_.y_)) -
                       ((other_line.end_.y_ - other_line.begin_.y_)*(begin_.x_ - other_line.begin_.x_));

        float nume_b = ((end_.x_ - begin_.x_)*(begin_.y_ - other_line.begin_.y_)) -
                       ((end_.y_ - begin_.y_)*(begin_.x_ - other_line.begin_.x_));

        if(denom == 0.0f)
        {
            if(nume_a == 0.0f && nume_b == 0.0f)
            {
                return COINCIDENT;
            }
            return PARALLEL;
        }

        float ua = nume_a / denom;
        float ub = nume_b / denom;

        if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
        {
            // Get the intersection point.
            intersection.x_ = begin_.x_ + ua*(end_.x_ - begin_.x_);
            intersection.y_ = begin_.y_ + ua*(end_.y_ - begin_.y_);

            return INTERESECTING;
        }

        return NOT_INTERESECTING;
    }
};

inline void DoLineSegmentIntersection(const Vector& p0, const Vector& p1, const Vector& p2, const Vector& p3)
{
    LineSegment linesegment0(p0, p1);
    LineSegment linesegment1(p2, p3);

    Vector intersection;

    std::cout << "Line Segment 0: (" << p0.x_ << ", " << p0.y_ << ") to (" << p1.x_ << ", " << p1.y_ << ")\n"
              << "Line Segment 1: (" << p2.x_ << ", " << p2.y_ << ") to (" << p3.x_ << ", " << p3.y_ << ")\n";

    switch(linesegment0.Intersect(linesegment1, intersection))
    {
    case LineSegment::PARALLEL:
        std::cout << "The lines are parallel\n\n";
        break;
    case LineSegment::COINCIDENT:
        std::cout << "The lines are coincident\n\n";
        break;
    case LineSegment::NOT_INTERESECTING:
        std::cout << "The lines do not intersect\n\n";
        break;
    case LineSegment::INTERESECTING:
        std::cout << "The lines intersect at (" << intersection.x_ << ", " << intersection.y_ << ")\n\n";
        break;
    }
}

} // namespace Coventry

namespace Geometry{

/// will return true if line segments (A,B) and (C,D) intersect
inline bool intersect(const Geometry::CartesianPoint& A, const Geometry::CartesianPoint& B, const Geometry::CartesianPoint& C, const Geometry::CartesianPoint& D)
{
    Coventry::Vector a(A.x, A.y);
    Coventry::Vector b(B.x, B.y);
    Coventry::Vector c(C.x, C.y);
    Coventry::Vector d(D.x, D.y);

    Coventry::LineSegment linesegment0(a, b);
    Coventry::LineSegment linesegment1(c, d);

    Coventry::Vector intersection;

    return linesegment0.Intersect(linesegment1, intersection) == Coventry::LineSegment::INTERESECTING;
}

/// will return true if line segments (A,B) and (C,D) intersect, GeographicPoint version.
inline bool intersect(const Geometry::GeographicPoint& A, const Geometry::GeographicPoint& B, const Geometry::GeographicPoint& C, const Geometry::GeographicPoint& D)
{
    Geometry::CartesianPoint a, b, c, d;
    a = geo2cart(A, A.lat);
    b = geo2cart(B, A.lat);
    c = geo2cart(C, A.lat);
    d = geo2cart(D, A.lat);

    return intersect(a, b, c, d);
}

/**
  This method is used to detect the first self-intersection in a list. Given list [1,2,3,4], the algorithm will check if
  segment (2,3) is intersecting with (1,2) or if (3,4) is intersecting (1,2) or (2,3). The algorithm will return an iterator pointing
  to the SECOND POINT of the first self-intersecting segment. So if (2,3) is intersecting (1,2), the iterator will point to 3. If the list
  is free of intersections, the returned iterator will point to lst.cend().
  */

inline CartesianPointList::const_iterator pointlist_selfintersecting(const Geometry::CartesianPointList& lst)
{

    std::cerr << "self-intersection test for list with " << lst.size() << " elements." << std::endl;

    if(lst.size() < 4)
        // less than three adjacent segments never intersect
        return lst.end();

    auto _lst = lst;

    CartesianPointList::iterator segment_second = _lst.begin();
    CartesianPointList::iterator segment_first = _lst.begin();
    CartesianPointList::iterator last_probe = _lst.begin();

    std::advance(last_probe, 1);
    std::advance(segment_first, 2);
    std::advance(segment_second, 3);

    bool intersection = false;

    for(; segment_second != _lst.end() && !intersection; ++segment_second, ++segment_first, ++last_probe)
    {
        CartesianPoint C = *segment_first;
        CartesianPoint D = *segment_second;

        pairwise_iterator< Geometry::CartesianPointList > first(_lst.begin());
        pairwise_iterator< Geometry::CartesianPointList > last(segment_first);

        for(; first != last && !intersection; ++first)
        {
            std::pair< Geometry::CartesianPoint, Geometry::CartesianPoint> pair = *first;
            CartesianPoint A = pair.first;
            CartesianPoint B = pair.second;
            intersection = intersect(A, B, C, D);
            std::cout << A.str() << ", " << B.str() << " INT " << C.str() << ", " << D.str() << "? " << (intersection ? "YES" : "NO") << std::endl;
        }
    }

    CartesianPointList::const_iterator it = lst.cbegin();
    std::advance(it, std::distance(_lst.begin(), segment_second));

    return it;
}


}


#endif // LINE_SEGMENT_INTERSECTION_HPP
