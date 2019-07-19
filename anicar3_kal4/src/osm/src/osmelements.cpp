#include <cstdlib>

#include <pugixml.hpp>
#include <bbf_geometry/geometry.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <deque>

#include "osmelements.h"

namespace OSM {
namespace {
class LocaleGuard {
public:
    LocaleGuard() {
#ifdef __linux__
        auto lc = localeconv();
        if (std::strcmp(lc->decimal_point, ".") != 0) {
            reset = true;
            oldLocale = setlocale(LC_NUMERIC, NULL);
            setlocale(LC_NUMERIC, "C");
        }
#endif
    }
    ~LocaleGuard() {
#ifdef __linux__
        if (reset) {
            setlocale(LC_NUMERIC, oldLocale.c_str());
        }
#endif
    }
    LocaleGuard(LocaleGuard&& rhs) noexcept = delete;
    LocaleGuard& operator=(LocaleGuard&& rhs) noexcept = delete;
    LocaleGuard(const LocaleGuard& rhs) = delete;
    LocaleGuard& operator=(const LocaleGuard& rhs) = delete;

private:
    bool reset{false};
    std::string oldLocale;
};

void tagsFromNode(Primitive& tagged, const pugi::xml_node& node) {

    for (auto tagNode = node.child("tag"); !!tagNode; tagNode = tagNode.next_sibling("tag")) {
        std::string key = tagNode.attribute("k").value();
        std::string value = tagNode.attribute("v").value();
        tagged.set_tag(key, value);
    }
}

void tagsToNode(const Primitive& tagged, pugi::xml_node& node) {
    for (const auto& tag : tagged.tags()) {
        auto tagNode = node.append_child("tag");
        tagNode.append_attribute("k") = tag.first.c_str();
        tagNode.append_attribute("v") = tag.second.c_str();
    }
}

void nodesFromDocument(PrimitiveMap& primitives, const pugi::xml_document& doc) {
    for (auto node = doc.child("osm").child("node"); !!node; node = node.next_sibling("node")) {
        auto attr = node.attribute("action");

        if (!!attr && strcmp(attr.value(), "delete") == 0) {
            continue;
        }

        std::string id = node.attribute("id").value();

        auto lon = node.attribute("lon").as_double(0.);
        auto lat = node.attribute("lat").as_double(0.);

        auto p = boost::make_shared<Node>(id, lat, lon);
        tagsFromNode(*p, node);

        primitives[p->identifier()] = p;
    }
}

void nodesToDocument(const PrimitiveMap& primitives, pugi::xml_node& osmNode) {
    for (const auto& prim : primitives) {
        auto node = boost::dynamic_pointer_cast<Node>(prim.second);
        if (!node) {
            continue;
        }
        auto xmlNode = osmNode.append_child("node");
        xmlNode.append_attribute("id") = node->id();
        xmlNode.append_attribute("lat") = node->lat;
        xmlNode.append_attribute("lon") = node->lon;
        if (node->id() > 0) {
            xmlNode.append_attribute("version") = 1;
        }
        tagsToNode(*node, xmlNode);
    }
}

void relationsFromDocument(PrimitiveMap& primitives, const pugi::xml_document& doc) {

    // two-pass-strategie is needed here: As relations can contain relations as members, all relations first need to be
    // known. Then the members can be assigned.


    std::deque<pugi::xml_node> toDoA, toDoB;


    for (auto node = doc.child("osm").child("relation"); !!node; node = node.next_sibling("relation")) {
        auto attr = node.attribute("action");

        if (!!attr && strcmp(attr.value(), "delete") == 0) {
            continue;
        }
        toDoB.push_back(node);
    }

    while (true) {
        toDoA = toDoB;
        toDoB.clear();

        for (auto node : toDoA) {
            osmid_t id = node.attribute("id").value();
            auto theRelation = boost::make_shared<Relation>(id);
            tagsFromNode(*theRelation, node);

            try {
                for (auto memberNode = node.child("member"); !!memberNode;
                     memberNode = memberNode.next_sibling("member")) {
                    std::string memberId = memberNode.attribute("ref").value();
                    std::string role = memberNode.attribute("role").value();
                    std::string type = memberNode.attribute("type").value();

                    PrimitivePtr theMember = primitives.at(std::make_pair(type, memberId));
                    theRelation->members.push_back(std::make_pair(role, theMember));
                }
                primitives[theRelation->identifier()] = theRelation;
            }

            catch (...) {
                // not able to build element
                toDoB.push_back(node);
            }
        }

        //    std::cerr << "a: " << to_do_A.size() << ", b: " << to_do_B.size() << std::endl;


        if (toDoA.size() == toDoB.size()) {
            // no item could be processed.
            break;
        }
    }

    //    std::cerr << "Finished building relations." << std::endl;
}

void relationsToDocument(const PrimitiveMap& primitives, pugi::xml_node& osmNode) {
    for (const auto& prim : primitives) {
        auto relation = boost::dynamic_pointer_cast<Relation>(prim.second);
        if (!relation) {
            continue;
        }
        auto xmlNode = osmNode.append_child("relation");
        if (relation->id() > 0) {
            xmlNode.append_attribute("version") = 1;
        }
        xmlNode.append_attribute("id") = relation->id();
        for (const auto& role : relation->members) {
            auto xmlMember = xmlNode.append_child("member");
            auto type = role.second->type_string();
            xmlMember.append_attribute("type") = type.c_str();
            xmlMember.append_attribute("ref") = role.second->id();
            xmlMember.append_attribute("role") = role.first.c_str();
        }
        tagsToNode(*relation, xmlNode);
    }
}

void waysFromDocument(PrimitiveMap& primitives, const pugi::xml_document& doc) {

    for (auto node = doc.child("osm").child("way"); !!node; node = node.next_sibling("way")) {
        auto attr = node.attribute("action");

        if (!!attr && strcmp(attr.value(), "delete") == 0) {
            continue;
        }

        std::string id = node.attribute("id").value();

        auto theWay = boost::make_shared<Way>(id);
        tagsFromNode(*theWay, node);

        try {
            for (auto wayNode = node.child("nd"); !!wayNode; wayNode = wayNode.next_sibling("nd")) {
                std::string ref = wayNode.attribute("ref").value();
                auto primitivePointer = primitives.at(std::make_pair("node", ref));
                NodePtr wp = boost::dynamic_pointer_cast<Node>(primitivePointer);
                theWay->nodes().push_back(wp);
                assert(theWay);
                primitives[theWay->identifier()] = theWay;
            }
        }

        catch (...) {
            // way is not complete: not building.
            continue;
        }
    }
}

void waysToDocument(const PrimitiveMap& primitives, pugi::xml_node& osmNode) {
    for (const auto& prim : primitives) {
        auto way = boost::dynamic_pointer_cast<Way>(prim.second);
        if (!way) {
            continue;
        }
        auto xmlNode = osmNode.append_child("way");
        if (way->id() > 0) {
            xmlNode.append_attribute("version") = 1;
        }
        xmlNode.append_attribute("id") = way->id();
        for (const auto& node : way->nodes()) {
            auto nd = xmlNode.append_child("nd");
            nd.append_attribute("ref") = node->id();
        }
        tagsToNode(*way, xmlNode);
    }
}

void parse(pugi::xml_document& doc, PrimitiveMap& primitives) {
    LocaleGuard g; // make sure . is correctly interpreted
    // from here, start parsing the XML tree.
    nodesFromDocument(primitives, doc);
    waysFromDocument(primitives, doc);
    relationsFromDocument(primitives, doc);
}

void write(pugi::xml_document& doc, const PrimitiveMap& primitives) {
    LocaleGuard g; // make sure  . is correctly written
    // from here, start parsing the XML tree.
    auto osmNode = doc.append_child("osm");
    osmNode.append_attribute("version") = "0.6";
    osmNode.append_attribute("generator") = "libosm";
    nodesToDocument(primitives, osmNode);
    waysToDocument(primitives, osmNode);
    relationsToDocument(primitives, osmNode);
}
} // namespace

void OSM::OSMHelper::parse_osm(const std::string& filename, PrimitiveMap& primitives) {
    pugi::xml_document doc;
    auto result = doc.load_file(filename.c_str());
    if (!result) {
        throw std::runtime_error(std::string("Errors occured while parsing osm file: ") + result.description());
    }
    parse(doc, primitives);
}

void OSMHelper::parse_osm(std::istream& stream, PrimitiveMap& primitives) {
    pugi::xml_document doc;
    auto result = doc.load(stream);
    if (!result) {
        throw std::runtime_error(std::string("Errors occured while parsing osm file: ") + result.description());
    }
    parse(doc, primitives);
}

void OSMHelper::write_osm(const std::string& filename, const PrimitiveMap& primitives) {
    pugi::xml_document xml;
    write(xml, primitives);
    xml.save_file(filename.c_str());
}

void OSMHelper::write_osm(std::ostream& stream, const PrimitiveMap& primitives) {
    pugi::xml_document xml;
    write(xml, primitives);
    xml.save(stream);
}

void OSM::OSMHelper::get_relations(const PrimitiveMap& primitives, RelationList& relations) {
    for (OSM::PrimitiveMap::const_iterator it = primitives.begin(); it != primitives.end(); ++it) {

        OSM::PrimitivePtr prim = it->second;

        if (prim && prim->type() == OSM::RELATION) {
            OSM::RelationPtr rel = boost::dynamic_pointer_cast<OSM::Relation>(prim);
            assert(rel);
            relations.push_back(rel);
        }
    }
}

Node::~Node() = default;
bool Node::operator==(const Primitive& other) const {
    auto node = dynamic_cast<const Node* const>(&other);
    if (node == nullptr) {
        return false;
    }
    auto near = [](double d1, double d2) { return std::abs(d1 - d2) < 1e-12; };
    return node->_id == this->_id && tags_ == node->tags_ && near(this->lat, node->lat) && near(this->lon, node->lon);
}

double Node::distance(double lat, double lon) {
    Geometry::GeographicPoint a(this->lat, this->lon);
    Geometry::GeographicPoint b(lat, lon);

    return Geometry::distance(a, b);
}

double Node::distance(const Node& other) {
    Geometry::GeographicPoint a(this->lat, this->lon);
    Geometry::GeographicPoint b(other.lat, other.lon);

    return Geometry::distance(a, b);
}

AbstractPrimitive::~AbstractPrimitive() = default;

Primitive::~Primitive() = default;

bool Relation::operator==(const Primitive& other) const {
    auto relation = dynamic_cast<const Relation*>(&other);
    if (relation == nullptr) {
        return false;
    }
    if (_id != relation->_id) {
        return false;
    }
    if (members.size() != relation->members.size()) {
        return false;
    }
    for (auto it1 = members.begin(), it2 = relation->members.begin(); it1 != members.end(); ++it1, ++it2) {
        if (it1->first != it2->first) {
            return false;
        }
        if (*it1->second != *it2->second) {
            return false;
        }
    }
    return tags_ == relation->tags_;
}

bool Way::operator==(const Primitive& other) const {
    auto way = dynamic_cast<const Way*>(&other);
    if (way == nullptr) {
        return false;
    }
    if (_id != way->_id) {
        return false;
    }
    if (nodes_.size() != way->nodes_.size()) {
        return false;
    }
    for (auto it1 = nodes_.begin(), it2 = way->nodes_.begin(); it1 != nodes_.end(); ++it1, ++it2) {
        if (**it1 != **it2) {
            return false;
        }
    }
    return tags_ == way->tags();
}

std::ostream& operator<<(std::ostream& stream, const Primitive& prim) {
    auto ident = prim.identifier();
    return stream << ident.first << ", id: " << ident.second;
}
} // namespace OSM
