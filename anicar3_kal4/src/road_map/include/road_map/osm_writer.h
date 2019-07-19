#pragma once
#include <fstream>
#include <memory>
#include <vector>

namespace OSM {

typedef int OsmPointId;
typedef int OsmWayId;
typedef int OsmRelationId;
typedef int OsmAreaId;
typedef std::pair<OsmWayId, std::vector<OsmPointId>> OsmWay;
typedef OsmWay OsmArea;
typedef std::tuple<OsmRelationId, std::vector<OsmWay>, std::vector<OsmWay>> OsmRelation;
typedef std::pair<std::string, std::string> OsmTag;

/**
 * @brief The OsmWriter class can be used to write objects in a correct notation to an osm-file
 */
class OsmWriter {
public:
    ~OsmWriter();
    /**
     * @brief OsmWriter initializer
     * @param filename name of file to write
     * @param lat0 initial fake gps pose (lateral)
     * @param lon0 initial fake gps pose (longitudinal)
     */
    OsmWriter(std::string filename);

    /**
     * @brief OsmWriter intializer for a generic filestream (can be stdout)
     * @param fs ready and initialized filestream
     * @param lat0 inital fake gps position
     * @param lon0 ..
     */
    OsmWriter(std::shared_ptr<std::ostream> fs);

    /**
     * @brief printPoint write a point to osm file
     * @param pt the point
     * @param tags tags for this point
     * @return id of the written point
     */
    OsmPointId printPoint(double lat, double lon, const std::vector<OsmTag>& osmTags = std::vector<OsmTag>());

    /**
     * @brief printWay prints a way in osm file
     * @param points
     * @param osmTags tags for the way
     * @return id of written way
     */
    OsmWayId printWay(const std::vector<OsmPointId>& points, const std::vector<OsmTag>& osmTags);

    /**
     * @brief printRelation prints a relation (e.g. corridor)
     * @param left left element of relation
     * @param right right element of relation
     * @return id of written relation
     */
    OsmRelationId printRelation(const std::vector<OsmWayId>& left, const std::vector<OsmWayId>& right,
                                const std::vector<OsmTag>& osmTags);

    /**
     * @brief printArea prints the area for
     * @param points that make the area (ordered!)
     * @return id of printed area
     */
    OsmAreaId printArea(const std::vector<OsmPointId>& points, const std::vector<OsmTag>& osmTags);

private:
    int objectCount;
    std::shared_ptr<std::ostream> fs;
};
typedef std::shared_ptr<OsmWriter> OsmWriter_ptr;
} // end of namespace markings
