#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <vector>
#include <utility>
#include <cassert>
#include <cmath>
#include <tuple>
#include <string>

#include <boost/math/special_functions.hpp>

namespace BBF {

struct PolygonIntersection
{
    double x, y, d;
};

struct Polygon {
    Polygon() {}

    Polygon( int size ): xs( size ), ys( size ) {}
    Polygon( const std::vector< double >& xs,  const std::vector< double >& ys ) : xs(xs), ys(ys) {  assert( xs.size() == ys.size() ); }

    bool point_in_polygon( double x, double y )  const;
    std::vector< PolygonIntersection > intersect( double x, double y, double theta ) const;

    std::string label;

    void reserve( int n )
    {
        xs.reserve( n );
        ys.reserve( n );
    }

    void resize( int n )
    {
        xs.resize( n );
        ys.resize( n );
    }

    void clear()
    {
        xs.clear();
        ys.clear();
    }

    void push_back( double x, double y )
    {  assert( xs.size() == ys.size() ); xs.push_back( x ); ys.push_back( y ); }

    void push_back( std::pair<double, double> p )
    {  assert( xs.size() == ys.size() ); xs.push_back( p.first ); ys.push_back( p.second ); }

    void extend( const Polygon& other )
    { xs.insert( xs.end(), other.xs.begin(), other.xs.end() );
        ys.insert( ys.end(), other.ys.begin(), other.ys.end() );
        assert( xs.size() == ys.size() ); }

    std::pair<double, double> at( int i ) const
    {  assert( xs.size() == ys.size() ); return std::make_pair( xs[i], ys[i] ); }

    int size() const
    { assert( xs.size() == ys.size() ); return xs.size(); }

    double distance( double x, double y ) const
    {
        const int N_VERT = xs.size() - 1;

        double dist = std::numeric_limits<double>::max();

        for( int i = 0; i < N_VERT; ++i )
        {
            double ax(xs[i]), ay(ys[i]), bx(xs[i+1]), by(ys[i+1]);
            double rx(bx - ax), ry(by - ay);
            double l(boost::math::hypot(rx, ry));
            double nx(-ry/l), ny(rx/l);
            double det(rx * ny + ry * nx);

            double a(ny/det), b(-nx/det), c(-ry/det), d(rx/det);

            double dx = x - ax;
            double dy = y - ay;

            double lambda_r = a*dx + b*dy;
            double lambda_n = c*dx + d*dy;

            if( lambda_r <= 0. )
            {
                // match to first point
                // dx, dy already fit

            }
            else if( lambda_r >= 1. )
            {
                // match to second point
                dx = x - bx;
                dy = y - by;
                return boost::math::hypot(dx, dy);
            }
            else
            {
                // match to segment
                double foot_x = ax + lambda_r * rx;
                double foot_y = ay + lambda_r * ry;

                dx = x - foot_x;
                dy = y - foot_y;
            }

            dist = std::min(dist, boost::math::hypot(dx, dy));
        }

        return dist;
    }

    std::vector<double> xs, ys;
};
}

inline std::vector< BBF::PolygonIntersection > BBF::Polygon::intersect( double x, double y, double theta ) const
{
    std::vector< BBF::PolygonIntersection > result;

    double u0, u1;
    u0 = std::cos( theta );
    u1 = std::sin( theta );

    // A, B: points of the segment, r0, r1: vector AB
    double A0, A1, B0, B1, r0, r1;

    // (S0, S1) = (x,y) + d * (u0, u1) = (A0, B0) + c * (r0, r1) = intersection point
    double c, d, S0, S1;

    for( int i = 0; i < this->size() -1; ++i )
    {
        std::tie(A0, A1) = this->at(i);
        std::tie(B0, B1) = this->at(i+1);

        r0 = B0 - A0;
        r1 = B1 - A1;

        c = (-A0*u1 + A1*u0 - u0*y + u1*x)/(r0*u1 - r1*u0);
        d = (-A0*r1 + A1*r0 + x*r1 - y*r0)/(r0*u1 - r1*u0);

        if( c >= 0 && c <= 1 )
        {
            S0 = x + d * u0;
            S1 = y + d * u1;
            result.push_back({S0, S1, d});
        }
    }

    return result;
}

inline bool BBF::Polygon::point_in_polygon( double x, double y ) const
{
    // taken from http://alienryderflex.com/polygon/

    int nvert = xs.size();
    bool odd_num_intersections = false;
    int i, j;

    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
        if ( ((ys[i]>y) != (ys[j]>y)) &&
             (x < (xs[j]-xs[i]) * (y-ys[i]) / (ys[j]-ys[i]) + xs[i]) )
            odd_num_intersections = !odd_num_intersections;
    }
    return odd_num_intersections;

}

#endif // POLYGON_HPP
