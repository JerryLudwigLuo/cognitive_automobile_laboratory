#include <boost/make_shared.hpp>
#include <cstring>
#include <stdio.h>
#include <string.h>
#include "osmelements.h"
#include "gtest/gtest.h"


using namespace OSM;
using namespace std::string_literals;

class OsmTest : public ::testing::Test {
public:
    void SetUp() override {
        primitives.clear();
        addNodes();
        addWays();
        addRelations();
    }

    void addNodes() {
        auto node = boost::make_shared<Node>("1"s, 1., 1.);
        node->set_tag("key", "value");
        addPrimitive(node);
        node = boost::make_shared<Node>("2"s, 2., 2.);
        node->set_tag("key", "value");
        addPrimitive(node);
        node = boost::make_shared<Node>("3"s, 3., 3.);
        node->set_tag("key", "value");
        addPrimitive(node);
    }

    void addWays() {
        auto way = boost::make_shared<Way>("1");
        way->set_tag("wayKey", "value");
        way->nodes().push_back(getNode("1"));
        way->nodes().push_back(getNode("2"));
        addPrimitive(way);

        way = boost::make_shared<Way>("2");
        way->set_tag("way2Key", "value");
        way->nodes().push_back(getNode("2"));
        way->nodes().push_back(getNode("1"));
        addPrimitive(way);
    }

    void addRelations() {
        auto relation = boost::make_shared<Relation>("2");
        relation->set_tag("relationKey", "anothervalue");
        relation->members.emplace_back("nodeRole"s, getNode("1"));
        relation->members.emplace_back("wayRole"s, getWay("1"));
        addPrimitive(relation);

        relation = boost::make_shared<Relation>("5");
        relation->set_tag("relation2Key", "anothervalue");
        relation->members.emplace_back("nodeRole"s, getNode("2"));
        relation->members.emplace_back("wayRole"s, getWay("2"));
        relation->members.emplace_back("relationRole"s, getRelation("2"));
        addPrimitive(relation);
    }

    void addPrimitive(const OSM::PrimitivePtr& prim) {
        primitives.insert(std::make_pair(prim->identifier(), prim));
    }

    NodePtr getNode(const std::string& id) {
        auto node = primitives[PrimitiveIdentifier("node", id)];
        return boost::dynamic_pointer_cast<Node>(node);
    }
    WayPtr getWay(const std::string& id) {
        auto node = primitives.at(PrimitiveIdentifier("way", id));
        return boost::dynamic_pointer_cast<Way>(node);
    }
    RelationPtr getRelation(const std::string& id) {
        auto node = primitives.at(PrimitiveIdentifier("relation", id));
        return boost::dynamic_pointer_cast<Relation>(node);
    }
    OSM::PrimitiveMap primitives;
};

template <typename C1, typename C2, typename Func> // NOLINT
void forEachPair(C1&& c1, C2&& c2, Func&& f) {
    auto it1 = c1.begin();
    auto it2 = c2.begin();
    for (; it1 != c1.end(); ++it1, ++it2) {
        f(*it1, *it2);
    }
}

TEST_F(OsmTest, readWrite) {
    std::string filename = std::tmpnam(nullptr);
    filename += ".osm";
    testing::FLAGS_gtest_catch_exceptions = false;
    OSM::OSMHelper::write_osm(filename, primitives);
    OSM::PrimitiveMap primitivesLoaded;
    OSM::OSMHelper::parse_osm(filename, primitivesLoaded);

    EXPECT_EQ(primitives.size(), primitivesLoaded.size());
    forEachPair(primitives, primitivesLoaded, [](const auto& c1, const auto& c2) {
        EXPECT_EQ(c1.first, c2.first);
        EXPECT_EQ(*c1.second, *c2.second);
    });
}

TEST_F(OsmTest, localeSettingsDoNotInfluenceResult) {
    std::stringstream ss;
    if (setlocale(LC_NUMERIC, "de_DE.UTF-8")) { // check only possible if locale "de_DE.UTF-8" is installed
        auto lc = localeconv();
        ASSERT_TRUE(std::strcmp(lc->decimal_point, ".") != 0) << lc->decimal_point; // comma separator is not "."
        ss << "<osm><node id=\"1\" lat=\"49\" lon=\"8.1\"/></osm>";
        OSM::PrimitiveMap primitivesLoaded;
        OSM::OSMHelper::parse_osm(ss, primitivesLoaded);
        EXPECT_EQ(1, primitivesLoaded.size());
        OSM::NodePtr node = boost::dynamic_pointer_cast<OSM::Node>(primitivesLoaded.at({"node", "1"}));
        EXPECT_DOUBLE_EQ(8.1, node->lon);
        auto lcNew = localeconv();
        ASSERT_TRUE(std::strcmp(lcNew->decimal_point, ".") != 0) << lcNew->decimal_point; // comma separator is not "."
    }
}
