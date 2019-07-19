#include "geometry.h"
#include <limits>
#include <limits>
#include <map>
#include <cmath>
#include <list>
#include "bbf_commons/pairwise_iterator.hpp"
#include <boost/functional/hash.hpp>
#include <boost/foreach.hpp>
#include "bbf_commons/LocalGeographicCS.hpp"
#include <tuple>
#include <stdexcept>
#include <boost/format.hpp>

#include "convex_hull.hpp"

#include <Eigen/LU>

using namespace Geometry;

void Geometry::offset( std::vector< Geometry::GeographicPoint >& pts, double amount )
{
    if( pts.empty() )
        return;

    auto first_point = pts.front();
    LocalGeographicCS cs(first_point.lat, first_point.lon );

    std::vector< double > xs_in, ys_in, xs_out, ys_out;
    xs_in.resize( pts.size() );
    ys_in.resize( pts.size() );

    for( size_t i = 0; i < pts.size(); ++i )
    {
        cs.ll2xy( pts[i].lat, pts[i].lon, xs_in[i], ys_in[i] );
    }

    offset_polyline(xs_in, ys_in, xs_out, ys_out, amount);

    for( size_t i = 0; i < pts.size(); ++i )
    {
        cs.xy2ll( xs_out[i], ys_out[i], pts[i].lat, pts[i].lon );
    }
}

double Geometry::heading(Geometry::GeographicPoint first, Geometry::GeographicPoint second)
{
    double mx0, mx1, my0, my1, delta_x, delta_y;

    convert_coordinates::latlon_to_scaled_mercator( first.lat, first.lon, first.lat, mx0, my0 );
    convert_coordinates::latlon_to_scaled_mercator( second.lat, second.lon, first.lat, mx1, my1 );

    delta_x = mx1 - mx0;
    delta_y = my1 - my0;

    return std::atan2(delta_y, delta_x);
}

Geometry::CartesianPoint my_projection(CartesianPoint p0, CartesianPoint p1, CartesianPoint q)
{
    // projects q onto line (p0, p1)

    Eigen::Matrix2d A;
    Eigen::Vector2d b;

    A << p1.x - p0.x, p1.y - p0.y, p0.y - p1.y, p1.x - p0.x;
    b << q.x*(p1.x - p0.x) + q.y * (p1.y - p0.y), p0.y*(p1.x - p0.x) - p0.x*(p1.y - p0.y);

    auto x = A.inverse() * b;

    return CartesianPoint(x(0), x(1));
}

Geometry::CartesianPoint Geometry::project( const Geometry::CartesianPoint& P,
                                           const Geometry::CartesianPointList& pts, double& u )
{
    if( pts.size() < 2 )
    {
        boost::format fmt("project(): Projection impossible for less than 2 points, got %i.");
        fmt = fmt % pts.size();
        throw std::runtime_error(fmt.str());
    }

    const size_t N_SEG = pts.size() - 1;
    // segment: "global" offset (first: 0), begin and end
    typedef std::tuple< double, CartesianPoint, CartesianPoint > segment_t;
    std::vector< segment_t > segments;

    double offset = 0;
    for(size_t i = 0; i < N_SEG; ++i)
    {
        segments.push_back( std::make_tuple( offset, pts[i], pts[i+1] ));
        offset += (pts[i+1] - pts[i]).norm();
    }

    // now: offset == line length (for later use)
    double min_dist = std::numeric_limits< double >::max();
    double arclen;
    CartesianPoint projected;

    bool found_one = false;

    BOOST_FOREACH( auto segm, segments )
    {
        double ooffset;
        CartesianPoint begin, end;
        std::tie(ooffset, begin, end) = segm;
        double uu;
        auto pprojected = project(begin, end, P, uu);

        // u E [0, 1]
        uu = std::min(uu, 1.0);
        uu = std::max(uu, 0.0);

        // correct projected point
        pprojected = begin + (end - begin) * uu;
        auto ddist = (P - pprojected).norm();

        if( ddist < min_dist )
        {
            found_one = true;
            min_dist = ddist;
            projected = pprojected;
            arclen = ooffset + (end - begin).norm()  * uu;
        }
    }

    if( !found_one )
        throw std::runtime_error("project(): No point selected.");

    u = arclen/offset;
    // std::cout << "[OFFSET] " << u << " " << min_dist << std::endl;

    return projected;
}

Geometry::GeographicPoint Geometry::project( const Geometry::GeographicPoint& P,
                                            const Geometry::GeographicPointVector& pts, double& u )
{
    LocalGeographicCS cs(P.lat, P.lon);
    CartesianPointList ppts;

    double x, y, lat, lon;
    cs.ll2xy(P.lat, P.lon, x, y);
    auto PP = CartesianPoint(x, y);

    BOOST_FOREACH( auto p, pts )
    {
        cs.ll2xy(p.lat, p.lon, x, y);
        ppts.push_back(CartesianPoint(x, y));
    }

    auto projected = Geometry::project(PP, ppts, u);
    cs.xy2ll(projected.x, projected.y, lat, lon);
    return Geometry::GeographicPoint(lat, lon);
}

double Geometry::distance( const CartesianPoint& A, const CartesianPoint& B, const CartesianPoint& C, bool _project )
{
    double u;

    Geometry::CartesianPoint projection_point = Geometry::project(A, B, C, u);

    if(u > 1 && !_project)
        return (B - C).norm();
    else if(u < 0 && !_project)
        return (A - C).norm();
    else
        return (projection_point - C).norm();
}

double Geometry::curvature(const GeographicPoint& P, const GeographicPointVector& points)
{

    std::map< double, GeographicPointVector::const_iterator > arclength_map;
    double u;

    for( auto it = points.cbegin(); it != points.cend(); ++it)
    {
        Geometry::project(*it, points, u);
        arclength_map[u] = it;
    }

    Geometry::project(P, points, u);

    // find the first element which has a greater arclength
    auto lb = arclength_map.lower_bound(u);

    Geometry::GeographicPoint pt = *(lb->second);
    Geometry::CartesianPoint p1 = Geometry::geo2cart(pt, P.lat);
    lb--;
    pt = *(lb->second);
    Geometry::CartesianPoint p2 = Geometry::geo2cart(pt, P.lat);

    CartesianPoint delta = p2 - p1;


    return std::atan2(delta(1), delta(0));
}


double Geometry::distance(const GeographicPoint& A, const GeographicPoint& B, const GeographicPoint& C, bool _project)
{
    double u;

    CartesianPoint _A = geo2cart(A, A.lat);
    CartesianPoint _B = geo2cart(B, A.lat);
    CartesianPoint _C = geo2cart(C, A.lat);

    Geometry::CartesianPoint _P = project(_A, _B, _C, u);

    if(_project || (u <= 1 && u >= 0))
        return (_P - _C).norm();

    else
    {
        if(u < 0)
            return (_C - _A).norm();
        else
            return (_C - _B).norm();
    }

}

Geometry::CartesianPoint Geometry::project(const CartesianPoint& A, const CartesianPoint& B, const CartesianPoint& C, double& u)
{
    CartesianPoint r1 = B - A;
    CartesianPoint r2(-r1[1], r1[0]);

    u = ( (C[0] - A[0]) * (B[0] - A[0] ) + (C[1] - A[1]) * (B[1] - A[1]) )/ std::pow((B -A).norm(), 2);

    CartesianPoint P =  A + (B - A) * u;
    return P;
}

double Geometry::distance(CartesianPoint C, CartesianPointList::const_iterator begin, CartesianPointList::const_iterator past_end)
{
    CartesianPointList::const_iterator itn, itf; // iterators 'first' and 'next',
    //consecutive iterators

    double min_dist = std::numeric_limits<double>::max();

    for(itn = begin, itf = itn++; itn != past_end; ++itf, ++itn)
    {
        CartesianPoint A = *itf;
        CartesianPoint B = *itn;

        double dist = distance(A, B, C);
        min_dist = std::min(dist, min_dist);
    }

    return min_dist;

}

double Geometry::distance(GeographicPoint gp, const GeographicPointVector& gpl)
{

    double min_dist = std::numeric_limits<double>::max();

    for(auto it = gpl.cbegin(); it != gpl.cend(); ++it)
    {
        double dst = Geometry::distance(gp, *it);
        min_dist = dst < min_dist ? dst : min_dist;
    }
    return min_dist;
}

double Geometry::distance(CartesianPoint C, const CartesianPointList& pl){
    return Geometry::distance(C, pl.cbegin(), pl.cend());
}

GeographicPointVector::iterator Geometry::get_next(const GeographicPoint& pos, GeographicPointVector& list)
{
    double min_dist = 1e10; //std::numeric_limits<double>::max();
    GeographicPointVector::iterator min_it;

    for(auto it = list.begin(); it != list.end(); ++it)
    {
        double d  = Geometry::distance(pos, *it);
        if(d < min_dist)
        {
            min_dist = d;
            min_it = it;
        }
    }

    return min_it;
}


Geometry::CartesianPoint Geometry::geo2cart(const GeographicPoint& p, double lat0){

    Geometry::GeographicPoint foo = p;

    double lon = foo.lon;
    double lat = foo.lat;

    double s = std::cos(M_PI * lat0 / 180.0);
    double mx = s * lon * M_PI * r_earth / 180;
    double my = s * r_earth * std::log(std::tan((M_PI * (90 + lat))/360));

    return Geometry::CartesianPoint(mx, my);
}

double foo(CartesianPointList list){
    double len = 0;
    pairwise_iterator< CartesianPointList > first(list.begin());
    pairwise_iterator< CartesianPointList > last(list.end());

    auto myfunc = [&len](const decltype(*first) the_pair)
    {
        Geometry::CartesianPoint A, B;
        A = the_pair.first;
        B = the_pair.second;
        len += (B -A).norm();};

    std::for_each(first, last, myfunc);
    return len;
}

double Geometry::length(const CartesianPointList& list){
    CartesianPointList::const_iterator it1, it2;
    double len = 0;
    for(it2 = list.begin(), it1 = it2, it2++; it2 != list.end(); ++it1, ++it2){
        CartesianPoint p1 = *it1;
        CartesianPoint p2 = *it2;
        len += (p2 - p1).norm();
    }
    return len;
}

double Geometry::length(const GeographicPointVector& list){
    GeographicPointVector::const_iterator it1, it2;
    double len = 0;
    for(it2 = list.begin(), it1 = it2, it2++; it2 != list.end(); ++it1, ++it2)
    {
        len += Geometry::distance(*it1, *it2);
    }

    return len;
}

GeographicPoint Geometry::cart2geo(const CartesianPoint& p, double lat0){


    double y = p[1];
    double x = p[0];

    double s = std::cos(M_PI * lat0 / 180.0);

    double lon = (x * 180) / (s * r_earth * M_PI);
    double lat = std::atan(std::exp(y/(r_earth * s))) * 360 / M_PI - 90;

    return GeographicPoint(lat, lon);
}

double Geometry::crossp_z(const Vector& a, const Vector& b){
    return a[0] * b[1] - a[1] * b[0];
}

void Geometry::rotate(Vector &a, double angle){
    double sa = std::sin(angle);
    double ca = std::cos(angle);

    double _a = a[0] * ca - a[1] * sa;
    double _b = a[0] * sa + a[1] * ca;

    a[0] = _a;
    a[1] = _b;

}

Eigen::Vector3d Geometry::transform_to_local_coordinates(double object_lat, double object_lon, double object_alt, double lat, double lon, double alt, double roll, double pitch, double yaw)
{
    //   USING_PART_OF_NAMESPACE_EIGEN

    double alpha, beta, gamma;

    gamma = roll;
    beta = pitch;
    alpha = yaw;

    Geometry::CartesianPoint car_pos = Geometry::geo2cart(Geometry::GeographicPoint(lat, lon), lat);
    Geometry::CartesianPoint object_pos = Geometry::geo2cart(Geometry::GeographicPoint(object_lat, object_lon), lat);

    Eigen::Vector3d C;
    C << car_pos(0) , car_pos(1), alt;

    Eigen::Vector3d P;
    P << object_pos(0), object_pos(1), object_alt;

    Eigen::Matrix3d rotation;

    double ca = std::cos(alpha);
    double cb = std::cos(beta);
    double cg = std::cos(gamma);

    double sa = std::sin(alpha);
    double sb=  std::sin(beta);
    double sg = std::sin(gamma);

    rotation << ca * cb , ca * sb * sg - sa * cg , ca * sb * cg + sa * sg ,
            sa * cb , sa * sb * sg + ca * cg , sa * sb - ca * sg ,
            -sb , cb * sg , cb * cg ;

    Eigen::Vector3d object_pos_in_car_coordinates = rotation * (P - C);
    return object_pos_in_car_coordinates;

}

Vector Geometry::rotate(const Vector& a, double angle){
    Vector b = a;
    rotate(b, angle);
    return b;
}

double Geometry::distance(const CartesianPoint& a, const CartesianPoint& b){
    CartesianPoint diff = a - b;
    return std::sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
}

double Geometry::distance(const GeographicPoint& a, const GeographicPoint& b){
    return distance( geo2cart(a, a.lat), geo2cart(b, a.lat) );
}

double Geometry::angle(const Vector &a, const Vector &b){

    int sign = Geometry::crossp_z(a, b) >= 0 ? 1 : -1;

    double _dot = a.dot(b);
    double _abs_a = a.norm();
    double _abs_b = b.norm();

    double _prod = _abs_a * _abs_b;
    double _ratio = _dot / _prod;

    _ratio = std::max(-1.0, std::min(1.0, _ratio));

    double _angle = std::acos(_ratio) * sign;

    return _angle;
}

double Geometry::angle(CartesianPoint A, CartesianPoint B, CartesianPoint C){
    const Vector a(B[0] - A[0], B[1] - A[1]);
    const Vector b(C[0] - A[0], C[1] - A[1]);

    return Geometry::angle(a, b);
}

bool Geometry::equal(const CartesianPoint& a, const CartesianPoint& b){
    return (b-a).norm() < 1e-3;
}
bool Geometry::equal(const GeographicPoint& a, const GeographicPoint& b){
    return Geometry::distance(a, b) < 1e-3;
}

std::size_t Geometry::hash_value(const Geometry::GeographicPoint& p)
{
    std::size_t seed = 0;
    boost::hash_combine(seed, p.lat);
    boost::hash_combine(seed, p.lon);

    return seed;
}

GeographicPoint Geometry::center(const GeographicPoint& a, const GeographicPoint& b){
    CartesianPoint A, B;
    A = geo2cart(a, a.lat);
    B = geo2cart(b, a.lat);

    A = (A + B) * 0.5;

    return cart2geo(A, a.lat);

}

CartesianPoint Geometry::center(const CartesianPoint& a, const CartesianPoint& b){
    return (a + b) * 0.5;
}

Geometry::LinearInterpolator::LinearInterpolator(const GeographicPointVector &pts)
{

}

GeographicPoint Geometry::LinearInterpolator::interpolate(double i)
{
    assert(false && "not implemented function!");
    return GeographicPoint();
}
