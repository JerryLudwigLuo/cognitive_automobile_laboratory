/*! \file   RoadMap.hpp
    \brief  Defines a GCDC road map consisting of trajectories of vertices.
    Author: Andreas Geiger, 2011
*/

#pragma once

#include <iostream>
#include <map>
#include <stdint.h>
#include <vector>
#include <ANN/ANN.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/bessel.hpp>
#include <memory>
#include "gnss_coordinate_transform/local_geographic_cs.hpp"

class RoadMap {

public:
    /*!
     * \brief constructor. First point of map is taken as reference point.
     * \param file filename to load map from
     * \throws invalid_argument if loading map fails
     */
    //explicit RoadMap(std::string file);

    /*!
     * \brief constructor. Reference point for coordinate transformation between lat/lon and cartesian values. can be
     * specified seperately. This reference point never changes through the lifetime of a road map. On construction, it
     * should be set to the current GPS latitude. All GPS positions in the vincinity of this latitude will be converted
     * correctly into metric cartesian coordinates.
     *
     * Note that trajectories shorter than 5 points will be ignored when using the search functions!
     * \param file filename to load map from
     * \param lat0 reference latitude for converting coordinates
     * \param lon0 reference longitude for converting coordinates
     * \throws invalid_argument if loading map fails
     */
    RoadMap(std::string file, double lat0, double lon0);

    /*!
     * \brief constructor
     * \param lat0 reference latitude for converting coordinates
     * \param lon0 reference longitude for converting coordinates
     */
    explicit RoadMap(double lat0 = 0.0, double lon0 = 0.0);

    /*! construct from a given set of vertices (for easily constructing test cases)
    RoadMap(const std::vector<double>& xs, const std::vector<double>& ys, double mapScale = 1.0);*/

    /*! add trajectory from a given set of vertices (for easily constructing test cases) */
    void addTrajectory(const std::vector<double>& xs, const std::vector<double>& ys);

    bool operator==(const RoadMap& map) const;
    bool operator!=(const RoadMap& map) const {
        return !(map == *this);
    };

    ~RoadMap();

    /*!
     * \brief inverse the direction of the map
     */
    void inverse();

    /*!
     * \brief convert lat/lon to cartesian meters using intrinsic scale
     * \param lat latitude in degrees
     * \param lon longitude in degrees
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     */
    void latLonToMeters(double lat, double lon, double& x, double& y) const;

    /*!
     * \brief convert cartesian meters to lat/lon using intrinsic scale
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \param lat latitude in degrees
     * \param lon longitude in degrees
     */
    void metersToLatLon(double x, double y, double& lat, double& lon) const;

    /*!
     * \brief add a new, empty trajectory to this map
     * \param id id for the new trajectory. will be cleared if existant. if negative, a value will be auto-generated
     * \returns id of new trajectory
     */
    int32_t newTrajectory(int32_t id = -1);

    /*!
     * \brief removes all vertices of the specified trajectory
     * \param t_id trajectory id (starting with 0)
     * \param erase erase trajectory
     * \returns true on success
     */
    bool clearTrajectory(int32_t t_id, bool erase = false);

    /*!
     * \brief removes all trajectories of this road map
     */
    void clearTrajectories();

    /*!
     * \brief trajectoryLength returns length of a trajectory or zero
     * \param t_id id of trajectory
     * \return length in meter
     */
    double trajectoryLength(int32_t t_id) const;

    /*!
     * \brief add a vertex (cartesian coordinate) to an existing trajectory
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \param t_id trajectory id (starting with 0)
     * \returns true on success
     */
    bool appendVertexMeters(double x, double y, int32_t t_id);

    /*!
     * \brief add a vertex (geographic coordinate) to an existing trajectory
     * \param lat latitude in degrees
     * \param lon longitude in degrees
     * \param t_id trajectory id (starting with 0)
     * \returns true on success
     */
    bool appendVertexLatLon(double lat, double lon, int32_t t_id);

    /*!
     * \brief retrieve vertex position as cartesian coordinate
     * \param t_id trajectory id
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \returns true on success
     */
    bool getVertexMeters(int32_t t_id, int32_t v_id, double& x, double& y) const;

    /*!
     * \brief retrieve vertex position as geographic coordinate
     * \param t_id trajectory id
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \param lat latitude in degrees
     * \param lon longitude in degrees
     * \returns true on success
     */
    bool getVertexLatLon(int32_t t_id, int32_t v_id, double& lat, double& lon) const;

    /*!
     * \brief retrieve vertex heading
     * \param t_id trajectory id
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \param heading heading in radiant
     * \returns true on success
     */
    bool getVertexHeading(int32_t t_id, int32_t v_id, double& heading) const;

    /*!
     * \brief retrieve trajectory and vertex id of vertex closest to given position
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \param t_id trajectory id (if t_id>=0 => used as input to force that t_id, t_id<0 => closest trajectory is
     * selected)
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \returns true on success
     */
    bool findClosestVertexMeters(double x, double y, int32_t& t_id, int32_t& v_id);

    /*!
     * \brief retrieve trajectory and vertex id of vertex closest to given position
     * \param lat latitude in degrees
     * \param lon longitude in degrees
     * \param t_id trajectory id (if t_id>=0 => used as input to force that t_id, t_id<0 => closest trajectory is
     * selected)
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \returns true on success
     */
    bool findClosestVertexLatLon(double lat, double lon, int32_t& t_id, int32_t& v_id);

    /*!
     * \brief retrieve geodesic travel distance and corresponding trajectory id / vertex id
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \param t_id trajectory id (if t_id>=0 => used as input to force that t_id, t_id<0 => closest trajectory is
     * selected)
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \param s scale of vector from v_id to v_id + 1 (p[v_id]+s*(p[v_id+1]-p[v_id]) gives the footpoint)
     * \param dist geodesic distance from trajectory start
     * \returns true on success
     */
    bool getGeodesicTravelDistanceMeters(double x, double y, int32_t& t_id, int32_t& v_id, double& s, double& dist);

    /*!
     * \brief retrieve geodesic travel distance and corresponding trajectory id / vertex id
     * \param lat latitude in degrees
     * \param lon longitude in degrees
     * \param t_id trajectory id (if t_id>=0 => used as input to force that t_id, t_id<0 => closest trajectory is
     * selected)
     * \param v_id vertex id within trajectory t_id (starting with 0)
     * \param s scale of vector from v_id to v_id + 1 (p[v_id]+s*(p[v_id+1]-p[v_id]) gives the footpoint)
     * \param dist geodesic distance from trajectory start
     * \returns true on success
     */
    bool getGeodesicTravelDistanceLatLon(double lat, double lon, int32_t& t_id, int32_t& v_id, double& s, double& dist);

    /*!
     * \brief get position from position and distance (takes closest trajectory)
     *        note: at the moment this function does no interpolation, which should be ok for visualization
     *              if you need more accurate outputs, please tell me ...
     * \param lat_in input latitude in degrees
     * \param lon_in input longitude in degrees
     * \param dist input distance in meters
     * \param lat_out output latitude
     * \param lon_out output longitude
     * \returns true on success (false if no ego trajectory found or output position of trajectory boundaries)
     */
    bool getPositionFromGeodesicDistance(double lat_in, double lon_in, double dist, double& lat_out, double& lon_out);

    /*!
     * \brief get position from given distance (note: x/y values will be interpolated)
     * \param t_id trajectory id
     * \param v_id vertex id within trajectory t_id (starting with 0) (should be nearby to improve speed)
     * \param dist input geodesic distance in meters (>= 0.0)
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \returns true on success
     */
    bool getPositionMetersFromGeodesicDistance(int32_t t_id, int32_t& v_id, double dist, double& x, double& y);

    /*!
     * \brief get footpoint of input point (lat, lon) on the closest trajectory
     *
     * \param lat_ego unused
     * \param lon_ego unused
     * \param lat input latitude in degrees
     * \param lon input longitude in degrees
     * \param foot_lat footpoint longitude
     * \param foot_lon footpoint longitude
     * \return true on success
     */
    bool getFootPoint(double lat_ego, double lon_ego, double lat, double lon, double& foot_lat, double& foot_lon);

    /*!
     * \brief get distance to, interpolated heading at and interpolated curvature at control
     * reference point (footpoint) with lookahead
     *
     * \note current version returns false if you are not 'within' start-end of trajectory
     *
     * \param lat_ego ego position latitude in degrees
     * \param lon_ego ego position longitude in degrees
     * \param heading ego heading
     * \param dist_lookAhead lookahead distance in meters
     * \param d_signed signed distance to reference point
     * \param heading_ref heading at reference point in radiant
     * \param curvature_ref curvature at reference point
     * \return true on success
     */
    bool getControlReferenceLookAhead(double lat_ego, double lon_ego, double heading, double dist_lookAhead,
                                      double& d_signed, double& heading_ref, double& curvature_ref);

    /*!
     * \brief get distance to, interpolated heading at and interpolated curvature at control
     * reference point (footpoint)
     *
     * \param x_ego ego position latitude in degrees
     * \param y_ego ego position longitude in degrees
     * \param d_signed signed distance to reference point
     * \param heading_ref heading at reference point in radiant
     * \param curvature_ref curvature at reference point
     * \param t_id trajectory id (if t_id>=0 => used as input to force that t_id, t_id<0 => closest
     * trajectory is selected)
     * \returns true on success
     */
    bool getControlReference(double lat_ego, double lon_ego, double& d_signed, double& heading_ref,
                             double& curvature_ref, int32_t t_id = -1);

    /*!
     * \brief get distance to, interpolated heading at and interpolated curvature at control
     * reference point (footpoint)
     *
     * \param x_ego ego position in meters (x is facing eastwards)
     * \param y_ego ego position in meters (y is facing north)
     * \param d_signed signed distance to reference point
     * \param heading_ref heading at reference point in radiant
     * \param curvature_ref curvature at reference point
     * \param t_id trajectory id (if t_id>=0 => used as input to force that t_id, t_id<0 => closest
     * trajectory is selected)
     * \returns true on success
     */
    bool getControlReferenceMeters(double x_ego, double y_ego, double& d_signed, double& heading_ref, double& curvature_ref, int32_t t_id = -1);

    /*!
     * \brief get curvature of given trajectory starting from v_id
     * \param t_id trajectory id
     * \param v_id id of first vertex in curvature result
     * \param horizon given in travel distance from first vertex
     * \param dist distance covered given in travel distance from first vertex
     *             (dist < horizon when end of trajectory has been reached)
     * \param this algorithm will append the curvature of each vertex from v_id to horizon to this vector
     * \return
     */
    bool getCurvature(int32_t t_id, int32_t v_id, double horizon, double& dist, std::vector<double> &curvature);

    /*!
     * \brief get number of trajectories
     * \returns all contained trajectory ids as a vector
     */
    std::vector<int32_t> numberOfTrajectories() const;

    /*!
     * \brief get number of vertices of trajectory t_id
     * \param t_id trajectory id of interest
     * \returns number of vertices
     */
    int32_t numberOfVertices(int32_t t_id) const;

    /*!
     * \brief load road map from file (scale + trajectories)
     * \param filename file to load from
     * \returns true on success
     */
    bool loadFromFile(std::string filename);

    /*!
     * \brief save road map to file (scale + trajectories)
     * \param filename file to save to
     */
    void saveToFile(std::string filename);

    /*!
     * \brief dumps kd-tree to file for plotting
     * \param filename file to save to
     */
    void dumpKdTreeToFile(std::string filename);

    /*!
     * \brief compute euclidean distance between 2 points
     * \param x1 x-coordinate of first point
     * \param y1 y-coordinate of first point
     * \param x2 x-coordinate of second point
     * \param y2 y-coordinate of second point
     */
    double l2dist(double x1, double y1, double x2, double y2) const;

    void smooth(int m = 6, double t = 2.0);

    // vertex (represented by cartesian coordinates in the map)
    struct vertex {
        double x, y;

        vertex() {
        }
        bool operator==(const vertex& comp) const {
            auto res = std::abs(x - comp.x) < 1e-5 && std::abs(y - comp.y) < 1e-5;
            //            if(!res) std::cout << "unequal vertex a: [" << x << ", " << y  << "], b: [" << comp.x << ", "
            //            << comp.y << "]" << std::endl;
            return res;
        }

        vertex(double x, double y) : x(x), y(y) {
        }
    };

    //double getMapScale() const;

    // private functions
    void buildCache();
    bool exists(int32_t id) const;
    bool shouldBeSaved() const;

    void deleteCache();

    int32_t numberOfVertices() const;

    /*!
     * \brief updates the travel distance dist by interpolating between vertices v_id1 and v_id2
     *
     * If the footpoint is not between v_id1 and v_id2 this function returns without changing v_id, s and dist
     *
     * \param t_id valid(!) trajectory id
     * \param v_id1 vertex id within trajectory t_id (starting with 0)
     * \param v_id2 vertex id within trajectory t_id (starting with 0)
     * \param x position in meters (x is facing eastwards)
     * \param y position in meters (y is facing north)
     * \param v_id v_id1
     * \param s scale of vector from v_id1 to v_id2 (p[v_id1]+s*(p[v_id2]-p[v_id1]) gives the footpoint)
     * \param dist geodesic distance from trajectory start to v_id1 plus v_id1 to footpoint distance
     */
    void updateTravelDistanceByInterpolation(int32_t t_id, int32_t v_id1, int32_t v_id2, double x, double y,
                                             int32_t& v_id, double& s, double& dist) const;


    // trajectories
    typedef std::vector<vertex> trajectory;
    std::map<int32_t, trajectory> trajectories;

    // cached distances of vertices along trajectories
    std::map<int32_t, std::vector<double>> distances;

    // cached kd tree for fast nearest neighbor search
    // there exists one kd tree per trajectory
    typedef std::shared_ptr<ANNkd_tree> ANNkd_tree_ptr;
    typedef std::shared_ptr<ANNpoint> ANNpointArray_ptr;
    struct kd_tree_struct {
        ANNkd_tree_ptr kd_tree;
        ANNpointArray_ptr kd_pts;
    };

    std::map<int32_t, kd_tree_struct> kd_trees;

 protected:
    gnss::LocalGeographicCS localGeographicCS;
    bool reference_point_set;
};
