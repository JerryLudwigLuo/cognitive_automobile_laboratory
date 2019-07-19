#include "osm_writer.h"
#include <cassert>
#include <iomanip>
#include <memory>

namespace {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
bool isGood(std::ostream* fs) {
    auto fsCast = dynamic_cast<std::ofstream*>(fs);
    return !fsCast || fsCast->is_open(); // dont check if cast fails (returns 0 because fs is no ofstream)
}
#pragma GCC diagnostic pop
}

namespace OSM {

OsmWriter::~OsmWriter() {
    *fs << "</osm>\n";
}

OsmWriter::OsmWriter(std::string filename) : objectCount{0}, fs(std::make_unique<std::ofstream>(filename)) {
    if (!dynamic_cast<std::ofstream*>(fs.get())->is_open())
        throw(std::runtime_error("could not open/create file " + filename));
    *fs << "<osm version=\"0.6\">\n";
    *fs << std::setprecision(12);
}

OsmWriter::OsmWriter(std::shared_ptr<std::ostream> fs) : objectCount{0}, fs(fs) {
    *this->fs << "<osm version=\"0.6\">\n";
    *this->fs << std::setprecision(12);
}

OsmPointId OsmWriter::printPoint(double lat, double lon, const std::vector<OsmTag>& osmTags) {
    assert(isGood(fs.get()));
    objectCount++;
    *fs << "<node lat=\"" << lat << "\" lon=\"" << lon << "\" id=\"-" << objectCount << "\" visible=\"true\">\n";
    *fs << "\t<tag k=\"real_id\" v=\"-" << objectCount << "\"/>\n";
    for (auto& tag : osmTags) {
        *fs << "\t<tag k=\"" << tag.first << "\" v=\"" << tag.second << "\"/>\n";
    }
    *fs << "</node>\n";

    return objectCount;
}

OsmWayId OsmWriter::printWay(const std::vector<OsmPointId>& points, const std::vector<OsmTag>& osmTags) {
    assert(isGood(fs.get()));
    if (points.empty())
        throw std::invalid_argument("cant print empty point vector");
    objectCount++;
    *fs << "<way id=\"-" << objectCount << "\">\n";
    for (auto& p : points)
        *fs << "\t<nd ref=\"-" << p << "\"/>\n";
    *fs << "\t<tag k=\"real_id\" v=\"-" << objectCount << "\"/>\n";
    *fs << "\t<tag k=\"shape\" v=\"solid\"/>\n";
    for (auto& tag : osmTags) {
        *fs << "\t<tag k=\"" << tag.first << "\" v=\"" << tag.second << "\"/>\n";
    }
    *fs << "</way>\n";
    return objectCount;
}

OsmRelationId OsmWriter::printRelation(const std::vector<OsmWayId>& left, const std::vector<OsmWayId>& right,
                                       const std::vector<OsmTag>& osmTags) {
    assert(isGood(fs.get()));
    if (left.empty())
        throw std::invalid_argument("cant print empty point vector");
    if (right.empty())
        throw std::invalid_argument("cant print empty point vector");

    objectCount++;
    *fs << "<relation id=\"-" << objectCount << "\">\n";
    for (auto& l : left) {
        *fs << "\t<member type=\"way\" ref=\"-" << l << "\" role=\"left\"/>\n";
    }
    for (auto& r : right) {
        *fs << "\t<member type=\"way\" ref=\"-" << r << "\" role=\"right\"/>\n";
    }
    *fs << "\t<tag k=\"real_id\" v=\"-" << objectCount << "\"/>\n";
    for (auto& tag : osmTags) {
        *fs << "\t<tag k=\"" << tag.first << "\" v=\"" << tag.second << "\"/>\n";
    }
    *fs << "</relation>\n";
    return objectCount;
}

OsmAreaId OsmWriter::printArea(const std::vector<OsmPointId>& points, const std::vector<OsmTag>& osmTags) {
    assert(isGood(fs.get()));
    if (points.empty())
        throw std::invalid_argument("cant print empty point vector");
    objectCount++;
    *fs << "<way id=\"-" << objectCount << "\">\n";
    *fs << "\t<tag k=\"real_id\" v=\"-" << objectCount << "\"/>\n";
    *fs << "\t<tag k=\"area\" v=\"yes\"/>\n";
    for (auto& tag : osmTags) {
        *fs << "\t<tag k=\"" << tag.first << "\" v=\"" << tag.second << "\"/>\n";
    }
    for (auto p : points) {
        *fs << "\t<nd ref=\"-" << p << "\"/>\n";
    }
    //    //close polygon if not yet happened
    //    if(points.size()>=2 && points.front() != points.back())
    //        *fs << "\t<nd ref=\"-" << points.front() <<"\"/>\n";
    *fs << "</way>\n";
    return objectCount;
}
} // end of namespace osm
