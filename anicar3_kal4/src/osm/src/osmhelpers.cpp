#include "osmhelpers.h"

using namespace OSM;

Geometry::GeographicPointVector OSM::adapt_nodelist(const NodeList& nodes) {
    Geometry::GeographicPointVector list;

    for (const auto& n : nodes) {
        list.push_back(Geometry::GeographicPoint(n->lat, n->lon));
    }

    return list;
}

Geometry::GeographicPoint OSM::adapt_node(Node node) {
    return Geometry::GeographicPoint(node.lat, node.lon);
}
