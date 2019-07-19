#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>
#include <map>

#include "bbf_commons/convert_coordinates.hpp"

#include <boost/operators.hpp>

#include <Eigen/Core>
#include <stdexcept>
#include <exception>
#include <string>

static const double r_earth = 6371e3;
static const double LAT0 = 49.246568;

namespace Geometry{

struct CartesianPoint : boost::addable<CartesianPoint, boost::subtractable<CartesianPoint> >
{
    double x, y;
    CartesianPoint(double _x, double _y) : x(_x), y(_y) { }
    CartesianPoint() : x(0), y(0) { }

    CartesianPoint operator- () const
    {
        CartesianPoint tmp(*this);
        tmp.x *= -1;
        tmp.y *= -1;
        return tmp;
    }

    CartesianPoint operator+ () const
    {
        return *this;
    }

    CartesianPoint& operator+= (const CartesianPoint& rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    CartesianPoint& operator -= (const CartesianPoint& rhs)
    {
        return operator+= (-rhs);
    }

    double norm() const
    {
        return std::sqrt( x*x + y*y );
    }

    double& operator()(int i)
    {
        if( i == 0)
            return x;
        return y;
    }

    double operator()(int i) const
    {
        if( i == 0)
            return x;
        return y;
    }

    double& operator[](int i)
    {
        if( i == 0)
            return x;
        return y;
    }

    double operator[](int i) const
    {
        if( i == 0)
            return x;
        return y;
    }

    double dot(const CartesianPoint& b) const
    {
        return this->x * b.x + this->y * b.y;
    }

    CartesianPoint operator/(double b) const
    {
        return CartesianPoint(this->x / b, this->y / b);
    }

    std::string str() const
    {
        std::stringstream strm;
        strm << "(" << x << ", " << y << ")";
        return strm.str();
    }


};

inline CartesianPoint operator*(const CartesianPoint& a, double b)
{
    return CartesianPoint(a.x * b, a.y * b);
}

struct GeographicPoint {
    double lat, lon;

    GeographicPoint() : lat(0), lon(0) {
    }
    GeographicPoint(const double& _lat, const double& _lon) : lat(_lat), lon(_lon) {
    }

    friend bool operator==(const GeographicPoint& lhs, const GeographicPoint& rhs) {
        double d_lat = std::abs(lhs.lat - rhs.lat);
        double d_lon = std::abs(lhs.lon - rhs.lon);
        return d_lat < 1e-6 && d_lon < 1e-6;
    }

    void shift_by(Geometry::CartesianPoint& other);
};

CartesianPoint geo2cart(const GeographicPoint& p, double lat0);
GeographicPoint cart2geo(const CartesianPoint& p, double lat0);

double distance( const GeographicPoint&, const GeographicPoint& );

template<typename T, typename V>
double distance(const T &first, const V &second)
{
    Geometry::GeographicPoint f(first.lat, first.lon);
    Geometry::GeographicPoint s(second.lat, second.lon);

    return distance(f, s);

}


double distance(const CartesianPoint&, const CartesianPoint&);


typedef CartesianPoint Vector;


inline Geometry::CartesianPoint difference(const Geometry::GeographicPoint& A, const Geometry::GeographicPoint& B)
{
    return (geo2cart(B, A.lat) - geo2cart(A, A.lat));
}



struct PolarCoordinatePoint
{
    double r, phi;
    PolarCoordinatePoint() : r(0), phi(0) {}
    PolarCoordinatePoint(double _r, double _phi) : r(_r), phi(_phi)
    {

    }
};

typedef std::vector< CartesianPoint > CartesianPointList;
typedef std::vector<GeographicPoint> GeographicPointVector;
typedef std::pair<GeographicPointVector, GeographicPointVector> GeographicPointListPair;
typedef std::vector<PolarCoordinatePoint> PolarCoordinateList;

double length(const CartesianPointList&);
double length(const GeographicPointVector&);

double curvature(const GeographicPoint& P, const GeographicPointVector& points);

double crossp_z(const CartesianPoint& a, const CartesianPoint& b);
void rotate(CartesianPoint& a, double angle);
CartesianPoint rotate(const CartesianPoint& a, double angle);

template<typename T>
struct Line{

    Line(){

    }

    T from;
    T to;
};

typedef Line<CartesianPoint> CartesianLine;
typedef std::vector<CartesianLine> CartesianLineList;

struct Box{

    Box(double xmin, double ymin, double xmax, double ymax){
        p_min = CartesianPoint(xmin, ymin);
        p_max = CartesianPoint(xmax, ymax);
    }

    Box(const CartesianPoint& pmin, const CartesianPoint& pmax){
        p_min = pmin;
        p_max = pmax;
    }

    Box(const CartesianPoint& p){
        p_min = p;
        p_max = p;
    }

    void extend(const CartesianPoint& p){
        p_min[0] = std::min(p[0], p_min[0]);
        p_min[1] = std::min(p[1], p_min[1]);

        p_max[0] = std::max(p[0], p_max[0]);
        p_max[1] = std::max(p[1], p_max[1]);
    }

    double diagonal() const{
        return (p_min - p_max).norm();
    }

    void print() const{
        std::cout << p_min.str() << " -> " << p_max.str() << std::endl;
    }
    CartesianPoint p_min, p_max;
};



double angle(CartesianPoint, CartesianPoint, CartesianPoint);
double angle(const CartesianPoint& a, const CartesianPoint& b);

Eigen::Vector3d transform_to_local_coordinates(double object_lat, double object_lon, double object_alt, double lat, double lon, double alt, double roll, double pitch, double yaw);

struct Pose{
    GeographicPoint pos;
    CartesianPoint orientation;

    Pose(const GeographicPoint& p, const CartesianPoint& o) : pos(p), orientation(o){
    }

};

bool equal(const CartesianPoint&, const CartesianPoint&);
bool equal(const GeographicPoint&, const GeographicPoint&);

GeographicPoint center(const GeographicPoint&, const GeographicPoint&);
CartesianPoint center(const CartesianPoint&, const CartesianPoint&);

void offset( std::vector< Geometry::GeographicPoint >& pts, double amount );


double heading(Geometry::GeographicPoint first, Geometry::GeographicPoint second);

/** distance point C - line segment (A, B). If project == true, the line segment will be extended if necessary, otherwise the distance cannot be bigger than the distance to A or B. */
double distance(const CartesianPoint& A, const CartesianPoint& B, const CartesianPoint& C, bool _project = false);
double distance(const GeographicPoint& A, const GeographicPoint& B, const GeographicPoint& C, bool _project = false);

/** project point C on line segment (A, B). If the scaling og the support vector is 0 or 1, the function will return A resp. B*/
Geometry::CartesianPoint project(const CartesianPoint& A, const CartesianPoint& B, const CartesianPoint& C, double& u);

Geometry::GeographicPoint project(const Geometry::GeographicPoint& P,
                                  const Geometry::GeographicPointVector& pts, double& u);

Geometry::CartesianPoint project(const Geometry::CartesianPoint& P,
                                  const Geometry::CartesianPointList& pts, double& u);


/** Distance Point <-> Polyline */
double distance(CartesianPoint, const CartesianPointList&);
double distance(GeographicPoint, const GeographicPointVector&);

/** Distance Point <-> Polyline, defined by iterators begin and past-end */
double distance(CartesianPoint, CartesianPointList::const_iterator, CartesianPointList::const_iterator);

std::size_t hash_value(const Geometry::GeographicPoint& );

GeographicPointVector::iterator get_next(const GeographicPoint& pos, GeographicPointVector& list);

struct LinearInterpolator
{

    LinearInterpolator(const GeographicPointVector&);
    GeographicPoint interpolate(double i);

private:

    GeographicPoint _ref;
    std::map< double, CartesianPoint > _segments;

};

inline void GeographicPoint::shift_by(CartesianPoint& other) {
    Geometry::CartesianPoint pt = geo2cart(*this, this->lat);
    pt += other;
    Geometry::GeographicPoint pt2 = cart2geo(pt, this->lat);

    this->lat = pt2.lat;
    this->lon = pt2.lon;
}

} // namespace Geometry




#endif // GEOMETRY_H
