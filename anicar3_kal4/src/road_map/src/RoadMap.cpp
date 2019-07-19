// Author: Andreas Geiger, 2011

#include "RoadMap.hpp"
#include <fstream>
#include <boost/filesystem.hpp>
#include <osm/osmelements.h>
#include <boost/property_tree/json_parser.hpp>
/*#include "gnss_coordinate_transform/convert_coordinates.hpp"*/
#include "osm_writer.h"

using namespace std;

/*RoadMap::RoadMap(std::string file) {
    reference_point_set = false;
    if (!loadFromFile(file.c_str()))
        throw invalid_argument("RoadMap: failed to load map [file: " + file + "]");
}*/

RoadMap::RoadMap(std::string file, double lat0, double lon0) {
    localGeographicCS.set_origin(lat0, lon0);
    reference_point_set = true;
    if (!loadFromFile(file.c_str()))
        throw invalid_argument("RoadMap: failed to load map [file: " + file + "]");
}

/*RoadMap::RoadMap(const vector<double>& xs, const vector<double>& ys, double mapScale) {
    double lat0 = std::acos(mapScale) * 180.0 / M_PI;
    double lat, lon;
    gnss::scaled_mercator_to_latlon(xs[0], ys[0], lat0, lat, lon);
    localGeographicCS.set_origin(lat, lon);
    reference_point_set = true;

    deleteCache();
    auto id = newTrajectory();
    auto& t = trajectories[id];
    for (size_t i = 0; i < xs.size(); i++) {
        t.push_back(vertex(xs[i], ys[i]));
    }

    buildCache();
}*/

RoadMap::RoadMap(double lat0, double lon0) {
    localGeographicCS.set_origin(lat0, lon0);
    reference_point_set = true;
}

void RoadMap::addTrajectory(const vector<double>& xs, const vector<double>& ys) {
    deleteCache();

    auto id = newTrajectory();
    auto& t = trajectories[id];
    for (size_t i = 0; i < xs.size(); i++) {
        t.push_back(vertex(xs[i], ys[i]));
    }

    buildCache();
}

bool RoadMap::operator==(const RoadMap& map) const {
    if (trajectories.size() != map.trajectories.size())
        return false;
    double lat0, lon0, lat1, lon1;
    localGeographicCS.getOrigin(lat0, lon0);
    map.localGeographicCS.getOrigin(lat1, lon1);
    if (lat0 != lat1 || lon0 != lon1)
        return false;
    for (auto it_this = trajectories.begin(), it_map = map.trajectories.begin(); it_this != trajectories.end();
         ++it_this, ++it_map) {
        if (it_this->first != it_map->first)
            return false;
        if (it_this->second != it_map->second)
            return false;
    }
    return true;
}

RoadMap::~RoadMap() {
    deleteCache();
}

void RoadMap::inverse() {
    deleteCache();

    // inverse each trajectory
    for (auto& t_entry : trajectories) {
        auto& t = t_entry.second;
        trajectory inv;
        inv.reserve(t.size());
        BOOST_REVERSE_FOREACH(vertex & v, t) {
            inv.push_back(v);
        }
        t = inv;
    }

    buildCache();
}

void RoadMap::latLonToMeters(double lat, double lon, double& x, double& y) const {
    localGeographicCS.ll2xy(lat, lon, x, y);
}

void RoadMap::metersToLatLon(double x, double y, double& lat, double& lon) const {
    localGeographicCS.xy2ll(x, y, lat, lon);
}

int32_t RoadMap::newTrajectory(int32_t id) {
    deleteCache();
    if (id < 0) {
        id = 0;
        while (exists(id))
            id++;
    }
    trajectories[id] = trajectory();
    return id;
}

bool RoadMap::clearTrajectory(int32_t t_id, bool erase) {
    deleteCache();
    if (!exists(t_id))
        return false;
    trajectories[t_id].clear();
    if (erase)
        trajectories.erase(trajectories.find(t_id));
    return true;
}

void RoadMap::clearTrajectories() {
    deleteCache();
    trajectories.clear();
}

double RoadMap::trajectoryLength(int32_t t_id) const
{
    if(!exists(t_id) || trajectories.at(t_id).empty()) return 0.;
    return distances.at(t_id).back();
}

bool RoadMap::appendVertexMeters(double x, double y, int32_t t_id) {
    deleteCache();
    if (!exists(t_id))
        return false;
    trajectories[t_id].push_back(vertex(x, y));
    return true;
}

bool RoadMap::appendVertexLatLon(double lat, double lon, int32_t t_id) {
    double x, y;
    latLonToMeters(lat, lon, x, y);
    return appendVertexMeters(x, y, t_id);
}

bool RoadMap::getVertexMeters(int32_t t_id, int32_t v_id, double& x, double& y) const {
    if (!exists(t_id))
        return false;
    if (v_id >= numberOfVertices(t_id))
        return false;
    x = trajectories.at(t_id)[v_id].x;
    y = trajectories.at(t_id)[v_id].y;
    return true;
}

bool RoadMap::getVertexLatLon(int32_t t_id, int32_t v_id, double& lat, double& lon) const {
    double x, y;
    bool success = getVertexMeters(t_id, v_id, x, y);
    if (success)
        metersToLatLon(x, y, lat, lon);
    return success;
}

bool RoadMap::getVertexHeading(int32_t t_id, int32_t v_id, double& heading) const {
    double x1, y1;
    bool success = getVertexMeters(t_id, v_id, x1, y1);
    if (success) {
        double x2, y2;
        if (getVertexMeters(t_id, v_id + 1, x2, y2)) { // use next point
            heading = atan2(y2 - y1, x2 - x1);
        } else {
            success = getVertexMeters(t_id, v_id - 1, x2, y2); // use previous point
            heading = atan2(y1 - y2, x1 - x2);
        }
    }
    return success;
}

bool RoadMap::findClosestVertexMeters(double x, double y, int32_t& t_id, int32_t& v_id) {

    // build kd trees if not existent
    if (kd_trees.size() == 0)
        buildCache();

    // if no trajectory id given => search all kd trees
    if (t_id < 0) {

        // allocate memory for query point
        ANNpoint query_pt = annAllocPt(2);
        ANNidxArray nn_idx = new ANNidx[1];
        ANNdistArray nn_dist = new ANNdist[1];
        query_pt[0] = x;
        query_pt[1] = y;

        // search closest trajectory and vertex
        t_id = -1;
        double min_dist = 1e100;
        for (auto kd_tree : kd_trees) {
            if (kd_tree.second.kd_tree != 0) {
                kd_tree.second.kd_tree->annkSearch(query_pt, 1, nn_idx, nn_dist);
                if (nn_dist[0] < min_dist) {
                    min_dist = nn_dist[0];
                    t_id = kd_tree.first;
                    v_id = nn_idx[0];
                }
            }
        }

        // deallocate memory for query point
        annDeallocPt(query_pt);
        delete[] nn_idx;
        delete[] nn_dist;

        // otherwise: search only the kd tree corresponding to t_id
    } else {

        // check if given trajectory id is valid
        if (kd_trees.find(t_id) == kd_trees.end())
            return false;
        if (kd_trees[t_id].kd_tree == 0)
            return false;

        // allocate memory for query point
        ANNpoint query_pt = annAllocPt(2);
        ANNidxArray nn_idx = new ANNidx[1];
        ANNdistArray nn_dist = new ANNdist[1];
        query_pt[0] = x;
        query_pt[1] = y;

        // search closest vertex
        kd_trees[t_id].kd_tree->annkSearch(query_pt, 1, nn_idx, nn_dist);
        v_id = nn_idx[0];

        // deallocate memory for query point
        annDeallocPt(query_pt);
        delete[] nn_idx;
        delete[] nn_dist;
    }

    v_id += 2; // adjust offset, cf. "buildCache"

    // return true on success
    if (t_id >= 0)
        return true;
    else
        return false;
}

bool RoadMap::findClosestVertexLatLon(double lat, double lon, int32_t& t_id, int32_t& v_id) {
    double x, y;
    latLonToMeters(lat, lon, x, y);
    return findClosestVertexMeters(x, y, t_id, v_id);
}

bool RoadMap::getGeodesicTravelDistanceMeters(double x, double y, int32_t& t_id, int32_t& v_id, double& s,
                                              double& dist) {
    if (kd_trees.size() == 0)
        buildCache();
    int32_t v_id_nn;
    if (!findClosestVertexMeters(x, y, t_id, v_id_nn))
        return false;
    dist = distances[t_id][v_id_nn];
    v_id = v_id_nn;
    s = 0;
    if (v_id_nn > 0)
        updateTravelDistanceByInterpolation(t_id, v_id_nn - 1, v_id_nn, x, y, v_id, s, dist);
    if (v_id_nn < numberOfVertices(t_id) - 1)
        updateTravelDistanceByInterpolation(t_id, v_id_nn, v_id_nn + 1, x, y, v_id, s, dist);
    return true;
}

bool RoadMap::getGeodesicTravelDistanceLatLon(double lat, double lon, int32_t& t_id, int32_t& v_id, double& s,
                                              double& dist) {
    double x, y;
    latLonToMeters(lat, lon, x, y);
    return getGeodesicTravelDistanceMeters(x, y, t_id, v_id, s, dist);
}

bool RoadMap::getPositionFromGeodesicDistance(double lat_in, double lon_in, double dist, double& lat_out,
                                              double& lon_out) {

    // convert input coordinates to meters
    double x_in, y_in;
    latLonToMeters(lat_in, lon_in, x_in, y_in);

    // find input trajectory, vertex id and distance
    int32_t t_id, v_id;
    double s, dist1;
    if (!getGeodesicTravelDistanceMeters(x_in, y_in, t_id, v_id, s, dist1))
        return false;

    // search forwards
    if (dist > 0) {

        double dist2 = dist1;
        while (dist2 < dist1 + dist) {
            v_id++;
            if (v_id >= numberOfVertices(t_id))
                return false;
            dist2 = distances[t_id][v_id];
        }

        // search backwards
    } else {

        double dist2 = dist1;
        while (dist2 > dist1 + dist) {
            v_id--;
            if (v_id < 0)
                return false;
            dist2 = distances[t_id][v_id];
        }
    }

    // return vertex
    return getVertexLatLon(t_id, v_id, lat_out, lon_out);
}

bool RoadMap::getPositionMetersFromGeodesicDistance(int32_t t_id, int32_t& v_id, double dist, double& x, double& y) {
    if (kd_trees.size() == 0)
        buildCache();
    if (kd_trees.size() == 0)
        return false;

    int N = numberOfVertices(t_id);
    if (N < 2 || dist < 0.0 || dist > distances[t_id][N - 1]) {
        return false;
    }
    if (v_id < 0 || v_id > N - 1) {
        // invalid index given, start from v_id = 0
        v_id = 0;
    }

    while (distances[t_id][v_id] < dist) {
        v_id++;
    }
    while (distances[t_id][v_id] > dist) {
        v_id--;
    }
    if (v_id > 0 && v_id == N - 1) {
        v_id = N - 2;
    }

    // extract vertex coordinates
    double x1 = trajectories[t_id][v_id].x;
    double y1 = trajectories[t_id][v_id].y;
    double x2 = trajectories[t_id][v_id + 1].x;
    double y2 = trajectories[t_id][v_id + 1].y;

    double vx = x2 - x1;
    double vy = y2 - y1;

    double d1 = distances[t_id][v_id];
    double d2 = distances[t_id][v_id + 1];

    double scale = (dist - d1) / (d2 - d1);
    x = x1 + scale * vx;
    y = y1 + scale * vy;

    return true;
}

bool RoadMap::getFootPoint(double lat_ego, double lon_ego, double lat, double lon, double& foot_lat, double& foot_lon) {

    int32_t t_id, v_id;
    double s, d;
    double x1, y1, x2, y2;

    // search on all trajectories
    t_id = -1;

    // retrieve v_id
    if (!getGeodesicTravelDistanceLatLon(lat, lon, t_id, v_id, s, d))
        return false;

    // get v_id vertex position in meters
    if (!getVertexMeters(t_id, v_id, x1, y1))
        return false;

    // first or last vertex id => return position of vertex directly
    if (v_id <= 0 || v_id >= numberOfVertices(t_id) - 1) {
        metersToLatLon(x1, y1, foot_lat, foot_lon);

        // somewhere in between => interpolate
    } else {

        // get v_id+1 position in meters
        if (!getVertexMeters(t_id, v_id + 1, x2, y2))
            return false;

        // interpolate
        double dx = x2 - x1;
        double dy = y2 - y1;
        metersToLatLon(x1 + s * dx, y1 + s * dy, foot_lat, foot_lon);
    }

    return true;
}

bool RoadMap::getControlReferenceLookAhead(double lat_ego, double lon_ego, double heading, double dist_lookAhead,
                                           double& d_signed, double& heading_ref, double& curvature_ref) {

    double x, y;
    latLonToMeters(lat_ego, lon_ego, x, y);

    x += dist_lookAhead * cos(heading);
    y += dist_lookAhead * sin(heading);

    return getControlReferenceMeters(x, y, d_signed, heading_ref, curvature_ref);
}

bool RoadMap::getControlReference(double lat_ego, double lon_ego, double& d_signed, double& heading_ref,
                                  double& curvature_ref, int32_t t_id) {
    double x, y;
    latLonToMeters(lat_ego, lon_ego, x, y);
    return getControlReferenceMeters(x, y, d_signed, heading_ref, curvature_ref, t_id);
}

bool RoadMap::getControlReferenceMeters(double x_ego, double y_ego, double& d_signed, double& heading_ref,
                                        double& curvature_ref, int32_t t_id) {
    int32_t v_id;                    // trajectory id, vertex id
    double s, d;                           // scale, dist
    double x0, y0, x1, y1, x2, y2, x3, y3; // 4 points (x1 = point directly before foot point)

    // get v_id/t_id
    if (!getGeodesicTravelDistanceMeters(x_ego, y_ego, t_id, v_id, s, d))
        return false;

    // get v_id-1 position in meters
    if (!getVertexMeters(t_id, v_id - 1, x0, y0))
        return false;

    // get v_id position in meters
    if (!getVertexMeters(t_id, v_id, x1, y1))
        return false;

    // get v_id+1 position in meters
    if (!getVertexMeters(t_id, v_id + 1, x2, y2))
        return false;

    // get v_id+2 position in meters
    if (!getVertexMeters(t_id, v_id + 2, x3, y3))
        return false;


    // compute foot point position
    double x_foot = x1 + s * (x2 - x1);
    double y_foot = y1 + s * (y2 - y1);

    // compute normalized tangent vector
    double vx = (x2 - x1);
    double vy = (y2 - y1);
    double norm = max(sqrt(vx * vx + vy * vy), 1e-20);
    vx /= norm;
    vy /= norm;

    // compute normal
    double nx = +vy;
    double ny = -vx;

    // compute lateral distance "d_signed"
    double px = x_ego - x1;
    double py = y_ego - y1;
    double distx = x_ego - x_foot;
    double disty = y_ego - y_foot;
    double sign = px * nx + py * ny >= 0 ? 1 : -1;
    d_signed = sign * sqrt(distx * distx + disty * disty);

    // derivatives
    double dx1 = 0.5 * (x2 - x0);
    double dy1 = 0.5 * (y2 - y0);
    double ddx1 = x2 - 2.0 * x1 + x0;
    double ddy1 = y2 - 2.0 * y1 + y0;

    double dx2 = 0.5 * (x3 - x1);
    double dy2 = 0.5 * (y3 - y1);
    double ddx2 = x3 - 2.0 * x2 + x1;
    double ddy2 = y3 - 2.0 * y2 + y1;

    // compute "heading"
    heading_ref = atan2(dy1 * (1. - s) + dy2 * s, dx1 * (1. - s) + dx2 * s);

    // compute "curvature"
    double denom1 = pow(dx1 * dx1 + dy1 * dy1, 3.0 / 2.0);
    if (fabs(denom1) < 1e-20)
        denom1 = 1e-20;
    double curvature_ref1 = (ddy1 * dx1 - dy1 * ddx1) / denom1;

    double denom2 = pow(dx2 * dx2 + dy2 * dy2, 3.0 / 2.0);
    if (fabs(denom2) < 1e-20)
        denom2 = 1e-20;
    double curvature_ref2 = (ddy2 * dx2 - dy2 * ddx2) / denom2;

    curvature_ref = curvature_ref1 * (1. - s) + curvature_ref2 * s;

    // yeah!
    return true;
}

bool RoadMap::getCurvature(
    int32_t t_id, int32_t v_id, double horizon, double& dist, std::vector<double>& curvatures) {
    if (kd_trees.size() == 0)
        buildCache();
    if (kd_trees.size() == 0)
        return false;

    int N = numberOfVertices(t_id);
    if (N < 3) {
        return false;
    }
    if (v_id < 1) {
        // invalid index given, start from v_id = 1
        v_id = 1;
    }
    if (v_id > N - 2) {
        // invalid index given, start from v_id = N - 2
        v_id = N - 2;
    }

    double x0, y0, x1, y1, x2, y2;
    double start_dist = distances[t_id][v_id];

    while (v_id + 1 < N && distances[t_id][v_id] < start_dist + horizon) {

        // get v_id-1 position in meters
        if (!getVertexMeters(t_id, v_id - 1, x0, y0))
            return false;

        // get v_id position in meters
        if (!getVertexMeters(t_id, v_id, x1, y1))
            return false;

        // get v_id+1 position in meters
        if (!getVertexMeters(t_id, v_id + 1, x2, y2))
            return false;

        // derivatives
        double dx1 = 0.5 * (x2 - x0);
        double dy1 = 0.5 * (y2 - y0);
        double ddx1 = x2 - 2.0 * x1 + x0;
        double ddy1 = y2 - 2.0 * y1 + y0;

        // compute "curvature"
        double denom = pow(dx1 * dx1 + dy1 * dy1, 3.0 / 2.0);
        if (fabs(denom) < 1e-20)
            denom = 1e-20;
        double curvature = (ddy1 * dx1 - dy1 * ddx1) / denom;

        // append it to the return value
        curvatures.push_back(curvature);

        // continue with next vertex
        v_id++;
    }

    dist = distances[t_id][v_id - 1];

    return true;
}

// i know, this can probably be found in a library, but i thought this would be overkill :)
inline void RoadMap::updateTravelDistanceByInterpolation(int32_t t_id, int32_t v_id1, int32_t v_id2, double x, double y,
                                                         int32_t& v_id, double& s, double& dist) const {

    // extract vertex coordinates
    double x1 = trajectories.at(t_id)[v_id1].x;
    double y1 = trajectories.at(t_id)[v_id1].y;
    double x2 = trajectories.at(t_id)[v_id2].x;
    double y2 = trajectories.at(t_id)[v_id2].y;

    // vectors to point and normalized vector along trajectory
    double px = x - x1;
    double py = y - y1;
    double vx = x2 - x1;
    double vy = y2 - y1;
    double norm = sqrt(vx * vx + vy * vy);
    if (fabs(norm) < 1e-20)
        return;
    vx /= norm;
    vy /= norm;

    // add additional distance (scalar product = projection)
    double scale = px * vx + py * vy;
    if (scale < 0 || scale > norm)
        return;

    // on success, set vertex id, vector scale and total geodesic distance
    v_id = v_id1;
    s = scale / norm;
    dist = distances.at(t_id)[v_id1] + scale;
}

std::vector<int32_t> RoadMap::numberOfTrajectories() const {
    std::vector<int32_t> ids;
    ids.reserve(trajectories.size());
    for (auto keyValue : trajectories)
        ids.push_back(keyValue.first);
    return ids;
}

int32_t RoadMap::numberOfVertices(int32_t t_id) const {
    if (!exists(t_id))
        return 0;
    return trajectories.at(t_id).size();
}

bool RoadMap::loadFromFile(string filename) {

    // load osm
    if (filename.substr(filename.find_last_of(".") + 1) == "osm") {
        OSM::PrimitiveMap osm_parser;
        try {
            OSM::OSMHelper::parse_osm(filename, osm_parser);
        } catch (std::exception& e) {
            std::cout << "osm parse error:" << e.what() << std::endl;
            return false;
        }
        // delete after try/catch to ensure exception safety
        deleteCache();

        for (auto it = osm_parser.cbegin(); it != osm_parser.cend(); ++it) {
            OSM::PrimitivePtr prim = it->second;
            OSM::WayPtr way = boost::dynamic_pointer_cast<OSM::Way>(prim);
            if (way) {
                int id = -1;
                if (way->has_tag("trajectory_id"))
                    id = std::stoi(way->get_tag("trajectory_id"));
                id = newTrajectory(id);
                const auto nodes = way->nodes();
                std::cout << "way(" << id << ") -> number of nodes: " << nodes.size() << std::endl;
                for (auto it2 = nodes.cbegin(); it2 != nodes.cend(); ++it2) {
                    // set scale for first time, because it is not saved in osm file
                    if (!reference_point_set) {
                        localGeographicCS.set_origin((*it2)->lat, (*it2)->lon);
                        reference_point_set = true;
                    }
                    appendVertexLatLon((*it2)->lat, (*it2)->lon, id);
                }
            }
        }
        buildCache();
        return true;
    } else {
        throw invalid_argument("RoadMap: json is not supported anymore, only osm; provided filename to load from: " + filename + " !");
        /*
        // default: load json
        boost::property_tree::ptree pt;
        try {
            boost::property_tree::read_json(filename, pt);
        } catch (...) {
            return false;
        }
        deleteCache();

        double mapScale = 0;
        for (auto entry : pt) {
            // catch scale, all other entries should be trajectories
            if (entry.first == "scale") {
                mapScale = entry.second.get_value<double>();
            } else {
                auto id = std::stoi(entry.first);
                newTrajectory(id);
                auto lat_entries = entry.second.get_child("lat");
                auto lon_entries = entry.second.get_child("lon");
                if (lat_entries.size() != lon_entries.size())
                    return false; // this is not really the user-friendly kind of error checking...
                for (auto it_lat = lat_entries.begin(), it_lon = lon_entries.begin(); it_lat != lat_entries.end();
                     ++it_lat, ++it_lon) {
                    auto lat = it_lat->second.get_value<double>();
                    auto lon = it_lon->second.get_value<double>();
                    if (!reference_point_set){
                      double lat0 = std::acos(mapScale) * 180.0 / M_PI;
                      localGeographicCS.set_origin(lat0, lon); //TODO Get lon0 from somewhere
                      reference_point_set = true;
                    }
                    appendVertexLatLon(lat, lon, id);
                }
            }
        }

        buildCache();
        return true;
        */
    }
}

// note: boost::archive::no_header ensures compatibility with smaller versions,
//       but may lead to trouble in the future ...
void RoadMap::saveToFile(string filename) {
    // dont save if there is no data (loading might fail when loading empty files)
    if (!shouldBeSaved())
        return;

    deleteCache();

    if(boost::filesystem::path(filename).parent_path().string() != std::string("")) {
        // create folders
        boost::filesystem::create_directories(boost::filesystem::path(filename).parent_path());
    }



    // save osm
    if (filename.substr(filename.find_last_of(".") + 1) == "osm") {
        OSM::OsmWriter osm_map(filename);
        for (auto& t_entry : trajectories) {
            if (t_entry.second.empty())
                continue;
            std::vector<OSM::OsmPointId> points;
            for (auto& p : t_entry.second) {
                double lat, lon;
                metersToLatLon(p.x, p.y, lat, lon);
                points.push_back(osm_map.printPoint(lat, lon));
            }
            osm_map.printWay(points,
                             {{{"name"}, {"trajectory"}}, {{"trajectory_id"}, {std::to_string(t_entry.first)}}});
        }
        return;
    }

    throw invalid_argument("RoadMap: json is not supported anymore, only osm; provided filename to save to: " + filename + " !");
    /*
    // default: save json
    using boost::property_tree::ptree;
    ptree pt;
    pt.put("scale", getMapScale());
    for (auto& t_entry : trajectories) {
        ptree t_pt, lat_pt, lon_pt;
        for (auto& p : t_entry.second) {
            double lat, lon;
            metersToLatLon(p.x, p.y, lat, lon);
            ptree lat_child, lon_child;
            // this is very inconvenient, but the only way to force boost to create an array...
            lat_child.put("", lat);
            lon_child.put("", lon);
            lat_pt.push_back(std::make_pair("", lat_child));
            lon_pt.push_back(std::make_pair("", lon_child));
        }
        t_pt.add_child("lat", lat_pt);
        t_pt.add_child("lon", lon_pt);
        pt.add_child(std::to_string(t_entry.first), t_pt);
    }
    boost::property_tree::write_json(filename, pt);

    buildCache();
    */
}

void RoadMap::dumpKdTreeToFile(std::string filename) {
    buildCache();
    ofstream file;
    file.open(filename);
    if (kd_trees.size() > 0 && kd_trees[0].kd_tree != 0)
        kd_trees[0].kd_tree->Dump(ANNtrue, file);
    file.close();
}

void RoadMap::buildCache() {

    // clear cache
    deleteCache();

    // create kd tree + cached distances
    for (auto t_it = trajectories.begin(); t_it != trajectories.end(); t_it++) {

        // reference to current trajectory
        trajectory& trajectory_curr = t_it->second;

        // init distance vector and kd tree
        vector<double> dist_vec;
        kd_tree_struct kd_tree;

        // construct kd tree if we have at least five points on this trajectory
        if (trajectory_curr.size() > 4) {

            // allocate points for kd tree (needs a deleter function...)
            kd_tree.kd_pts = std::shared_ptr<ANNpoint>(annAllocPts(trajectory_curr.size() - 4, 2), [](ANNpointArray p) { annDeallocPts(p); });

            // fill points and distance vector
            double dist = 0;
            double x_old = trajectory_curr.begin()->x;
            double y_old = trajectory_curr.begin()->y;
            dist_vec.push_back(dist);
            for (size_t v_id = 1; v_id < trajectory_curr.size(); v_id++) {
                if( v_id >= 2 && v_id < trajectory_curr.size()-2){
                kd_tree.kd_pts.get()[v_id - 2][0] = trajectory_curr[v_id].x;
                kd_tree.kd_pts.get()[v_id - 2][1] = trajectory_curr[v_id].y;
                }
                dist += l2dist(trajectory_curr[v_id].x, trajectory_curr[v_id].y, x_old, y_old);
                dist_vec.push_back(dist);
                x_old = trajectory_curr[v_id].x;
                y_old = trajectory_curr[v_id].y;
            }

            // allocate kd tree
            kd_tree.kd_tree = std::make_shared<ANNkd_tree>(kd_tree.kd_pts.get(), trajectory_curr.size() - 4, 2);

            // push back
            distances[t_it->first] = dist_vec;
            kd_trees[t_it->first] = kd_tree;
        }
    }
}

bool RoadMap::exists(int32_t id) const {
    return trajectories.find(id) != trajectories.end();
}

bool RoadMap::shouldBeSaved() const {
    return numberOfVertices();
}

void RoadMap::deleteCache() {
    distances.clear();
    kd_trees.clear();
    annClose();
}

int32_t RoadMap::numberOfVertices() const {
    int32_t num_vertices = 0;
    for (auto t : trajectories)
        num_vertices += t.second.size();
    return num_vertices;
}

double RoadMap::l2dist(double x1, double y1, double x2, double y2) const {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

void RoadMap::smooth(int m, double t) {
    double kernel[m * 2 + 1];
    double kernel_sum = 0.;
    for (int i = -m; i < m + 1; i++) {
        int n = i + m;
        kernel[n] = exp(-t) * boost::math::cyl_bessel_i(i, t);
        kernel_sum += kernel[n];
        cout << "kernel " << kernel[n] << "\n";
    }

    for (auto& t_entry : trajectories) {
        trajectory smoothed;
        auto& traj = t_entry.second;

        for (size_t i = m; i < traj.size() - m; i++) {
            double x = 0.;
            double y = 0.;

            for (int j = -m; j < m + 1; j++) {
                x += kernel[j + m] * traj[i + j].x;
                y += kernel[j + m] * traj[i + j].y;
            }

            smoothed.push_back(vertex(x / kernel_sum, y / kernel_sum));
        }

        traj = smoothed;
    }
}

/*double RoadMap::getMapScale() const {
    double lat, lon;
    localGeographicCS.getOrigin(lat, lon);
    return gnss::lat_to_scale(lat);
}*/
