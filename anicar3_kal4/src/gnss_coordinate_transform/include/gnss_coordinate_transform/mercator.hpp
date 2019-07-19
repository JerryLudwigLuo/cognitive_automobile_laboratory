#pragma once

#ifndef REALLY_USE_OLD_MERCATOR
#error "Local Mercator is not used anymore at MRT. If you really need the old behavior please #define REALLY_USE_OLD_MERCATOR."
#endif

#include <math.h>

/*
 * Mercator transformation
 * accounts for the fact that the earth is not a sphere, but a spheroid
 *
 * Taken from
 * http://wiki.openstreetmap.org/wiki/Mercator#C_implementation
 *
 */
#define D_R (M_PI / 180.0)
#define R_D (180.0 / M_PI)
#define R_MAJOR 6378137.0
#define R_MINOR 6356752.3142
#define RATIO (R_MINOR / R_MAJOR)
#define ECCENT (sqrt(1.0 - (RATIO * RATIO)))
#define COM (0.5 * ECCENT)

namespace gnss {

inline static double deg_rad(double ang) {
    return ang * D_R;
}

inline double merc_x(double lon) {
    return R_MAJOR * deg_rad(lon);
}

inline double merc_y(double lat) {
    lat = fmin(89.5, fmax(lat, -89.5));
    double phi = deg_rad(lat);
    double sinphi = sin(phi);
    double con = ECCENT * sinphi;
    con = pow((1.0 - con) / (1.0 + con), COM);
    double ts = tan(0.5 * (M_PI * 0.5 - phi)) / con;
    return 0 - R_MAJOR * log(ts);
}

inline static double rad_deg(double ang) {
    return ang * R_D;
}

inline double merc_lon(double x) {
    return rad_deg(x) / R_MAJOR;
}

inline double merc_lat(double y) {
    double ts = exp(-y / R_MAJOR);
    double phi = M_PI_2 - 2 * atan(ts);
    double dphi = 1.0;
    for (int i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) {
        double con = ECCENT * sin(phi);
        dphi = M_PI_2 - 2 * atan(ts * pow((1.0 - con) / (1.0 + con), COM)) - phi;
        phi += dphi;
    }
    return rad_deg(phi);
}

inline std::vector<double> merc_x(std::vector<double> lon) {
    std::vector<double> x(lon.size());
    for (size_t i = 0; i < lon.size(); i++) {
        x[i] = merc_x(lon[i]);
    }
    return x;
}

inline std::vector<double> merc_y(std::vector<double> lat) {
    std::vector<double> y(lat.size());
    for (size_t i = 0; i < lat.size(); i++) {
        y[i] = merc_y(lat[i]);
    }
    return y;
}

inline std::vector<double> merc_lat(std::vector<double> y) {
    std::vector<double> lat(y.size());
    for (size_t i = 0; i < y.size(); i++) {
        lat[i] = merc_lat(y[i]);
    }
    return lat;
}

inline std::vector<double> merc_lon(std::vector<double> x) {
    std::vector<double> lon(x.size());
    for (size_t i = 0; i < x.size(); i++) {
        lon[i] = merc_lon(x[i]);
    }
    return lon;
}
}
