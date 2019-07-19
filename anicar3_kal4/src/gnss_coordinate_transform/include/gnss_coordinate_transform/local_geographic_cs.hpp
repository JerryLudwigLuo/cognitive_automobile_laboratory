
// File:           LocalGeographicCS.hpp
// Creation Date:  Tuesday, March  6 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#pragma once

#include <assert.h>
#include <exception>
#include <limits>
#include <utility>

#include <GeographicLib/UTMUPS.hpp>


namespace gnss {

struct LocalGeographicCS {
    LocalGeographicCS();
    LocalGeographicCS(double lat0, double lon0, bool useOffset = true);

    void set_origin(double lat0, double lon0, bool useOffset = true);
    void getOrigin(double& lat0, double& lon0) const;

    void ll2xy(double lat, double lon, double& x, double& y) const;
    void xy2ll(double x, double y, double& lat, double& lon) const;

    std::pair<double, double> ll2xy(double lat, double lon) const;
    std::pair<double, double> xy2ll(double x, double y) const;

    // operate on containers
    template <class ItIn, class ItOut>
    void ll2xy(const ItIn& lat_begin,
               const ItIn& lat_end,
               const ItIn& lon_begin,
               const ItOut& x_begin,
               const ItOut& y_begin) const;

    template <class ItIn, class ItOut>
    void xy2ll(const ItIn& x_begin,
               const ItIn& x_end,
               const ItIn& y_begin,
               const ItOut& lat_begin,
               const ItOut& lon_begin) const;

private:
    int zone_;
    bool isInNorthernHemisphere_, useOffset_;
    double lat0_{0}, lon0_{0}, xOffset_{0}, yOffset_{0};
};

inline LocalGeographicCS::LocalGeographicCS(double lat0, double lon0, bool useOffset) {
    set_origin(lat0, lon0, useOffset);
}

inline LocalGeographicCS::LocalGeographicCS() {
    set_origin(0, 0);
}

inline void LocalGeographicCS::set_origin(double lat0, double lon0, bool useOffset) {
    lat0_ = lat0;
    lon0_ = lon0;
    useOffset_ = useOffset;
    double x, y;
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(lat0, lon0, zone, northp, x, y);
    if (useOffset_) {
        xOffset_ = x;
        yOffset_ = y;
    }
    zone_ = zone;
    isInNorthernHemisphere_ = northp;
}

inline void LocalGeographicCS::ll2xy(double lat, double lon, double& x, double& y) const {
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
    if (useOffset_) {
        x -= xOffset_;
        y -= yOffset_;
    }
    if (zone != zone_) {
        throw std::range_error("You have left the UTM zone!");
    }
    if (northp != isInNorthernHemisphere_) {
        throw std::range_error("You have changed the hemisphere!");
    }
}

inline std::pair<double, double> LocalGeographicCS::ll2xy(double lat, double lon) const {
    double x, y;
    ll2xy(lat, lon, x, y);
    return std::make_pair(x, y);
}

inline void LocalGeographicCS::xy2ll(double x, double y, double& lat, double& lon) const {
    if (useOffset_) {
        x += xOffset_;
        y += yOffset_;
    }
    GeographicLib::UTMUPS::Reverse(zone_, isInNorthernHemisphere_, x, y, lat, lon);

    // for zone compliance testing:
    double xTest, yTest;
    ll2xy(lat, lon, xTest, yTest);
}

inline std::pair<double, double> LocalGeographicCS::xy2ll(double x, double y) const {
    double lat, lon;
    xy2ll(x, y, lat, lon);
    return std::make_pair(lat, lon);
}

// operate on containers
template <class ItIn, class ItOut>
void LocalGeographicCS::ll2xy(const ItIn& lat_begin,
                              const ItIn& lat_end,
                              const ItIn& lon_begin,
                              const ItOut& x_begin,
                              const ItOut& y_begin) const {
    ItIn lat = lat_begin;
    ItIn lon = lon_begin;
    ItOut x = x_begin;
    ItOut y = y_begin;

    for (; lat != lat_end; lat++, lon++, x++, y++) {
        ll2xy(*lat, *lon, *x, *y);
    }
}

template <class ItIn, class ItOut>
void LocalGeographicCS::xy2ll(
    const ItIn& x_begin, const ItIn& x_end, const ItIn& y_begin, const ItOut& lat_begin, const ItOut& lon_begin) const {
    ItIn x = x_begin;
    ItIn y = y_begin;

    ItOut lat = lat_begin;
    ItOut lon = lon_begin;

    for (; x != x_end; lat++, lon++, x++, y++)
        xy2ll(*x, *y, *lat, *lon);
}

inline void LocalGeographicCS::getOrigin(double& lat0, double& lon0) const {
    lat0 = lat0_;
    lon0 = lon0_;
}
}
