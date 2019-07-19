#ifndef OSMHELPERS_H
#define OSMHELPERS_H

#include "osmelements.h"
#include "bbf_geometry/geometry.h"

namespace OSM {

Geometry::GeographicPointVector adapt_nodelist(const NodeList& nodes);
Geometry::GeographicPoint adapt_node(Node node);

} // namespace OSM

#endif // OSMHELPERS_H
