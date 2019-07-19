#pragma once

#include <cassert>

template< typename T >
bool inrange(T val, const T& lo, const T& hi)
{
    return val >= lo && val <= hi;
}

void assert_point( double lat, double lon )
{
    assert( inrange(lat, -90., 90.)  && "lat in sensible range");
    assert( inrange(lon, -180., 180.) && "lon in sensible range" );
}

