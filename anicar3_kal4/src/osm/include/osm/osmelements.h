#ifndef OSMELEMENTS_H
#define OSMELEMENTS_H

#include <list>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>


namespace OSM {

struct NoSuchMember {
    int32_t relation_id;
    std::string requested_member;
};

class TagError : public std::runtime_error {
public:
    explicit TagError(const std::string& message) : std::runtime_error(message) {
    }
};

using bounding_box_t = std::tuple<double, double, double, double>;

enum type_t { NODE = 0, WAY = 1, RELATION = 2 };

using osmid_t = std::string;
using PrimitiveIdentifier = std::pair<std::string, osmid_t>;

template <typename T>
struct Ptr {
    using Type = boost::shared_ptr<T>;
};

template <typename T>
struct List {
    using Type = std::list<typename Ptr<T>::Type>;
};

template <typename T>
struct Vector {
    using Type = std::vector<typename Ptr<T>::Type>;
};

typedef std::map<std::string, std::string> TagMap;

// interface each primitive has to provide
class AbstractPrimitive {

public:
    virtual type_t type() const = 0;
    virtual std::string get_tag(std::string key, std::string dflt = "none") const = 0;
    virtual bool has_tag(std::string key) = 0;
    virtual void set_tag(std::string key, std::string value) = 0;
    virtual TagMap tags() const = 0;
    virtual ~AbstractPrimitive();

    virtual int id() = 0;
};

class Primitive : public AbstractPrimitive {

public:
    Primitive(const osmid_t& id, type_t tp) : _id(id), _type(tp) {
    }
    ~Primitive() override;

    const osmid_t _id;
    boost::optional<int32_t> _id_num;

    virtual bool operator==(const Primitive& other) const = 0;
    bool operator!=(const Primitive& other) const {
        return !(*this == other);
    }


    int id() override {
        if (!_id_num) {
            _id_num = std::stoi(_id);
        }
        return *_id_num;
    }

    PrimitiveIdentifier identifier() const {
        return std::make_pair(this->type_string(), this->_id);
    }

    virtual bounding_box_t bb() = 0;

    std::string type_string() const {
        type_t tp = type();
        if (tp == NODE) {
            return "node";
        }
        if (tp == WAY) {
            return "way";
        } else {
            return "relation";
        }
    }

    std::string get_tag(std::string key, std::string dflt = "none") const override {
        std::map<std::string, std::string>::const_iterator pos = this->tags_.find(key);
        if (pos == this->tags_.end()) {
            return dflt;
        } else {
            return (*pos).second;
        }
    }

    bool has_tag(std::string key) override {
        return this->tags_.find(key) != this->tags_.end();
    }

    void set_tag(std::string key, std::string value) override {
        this->tags_[key] = value;
    }

    TagMap tags() const override {
        return this->tags_;
    }

protected:
    TagMap tags_;
    const type_t _type;
};

using PrimitivePtr = Ptr<Primitive>::Type;

class Node : public Primitive {

public:
    Node(const osmid_t& id, double lat, double lon) : Primitive(id, NODE), lat(lat), lon(lon) {
    }

    ~Node() override;
    bool operator==(const Primitive& other) const override;

    bounding_box_t bb() override {
        return std::make_tuple(lat, lon, lat, lon);
    }

    double distance(double lat, double lon);

    double distance(const Node& other);

    std::string to_string() {
        return "(" + std::to_string(lat) + ", " + std::to_string(lon) + ")";
    }

    type_t type() const override {
        return NODE;
    }

    double lat, lon;
};

class Way;
using NodePtr = Ptr<Node>::Type;
using NodeList = Vector<Node>::Type;
using WayList = List<Way>::Type;

class AbstractWay {

public:
    virtual OSM::NodeList nodes() const = 0;
    virtual OSM::NodePtr first() const {
        return this->nodes().front();
    }

    virtual OSM::NodePtr last() const {
        return this->nodes().back();
    }

    virtual std::string tag(std::string key) = 0;

    int size() {
        return int(this->nodes().size());
    }

    OSM::NodePtr node_at(int position) {
        assert(position < this->size() && "index needs to be less than listsize");
        auto nds = this->nodes();
        auto begin = nds.cbegin();
        std::advance(begin, position);
        return *begin;
    }

    double length() {
        auto nds = this->nodes();
        return length(nds.cbegin(), nds.cend());
    }

    double length(OSM::NodeList::const_iterator begin, OSM::NodeList::const_iterator end) {

        if (begin == end) {
            return 0;
        }


        double len = 0;
        auto pos = begin;
        auto next = pos;
        next++;

        for (; next != end; ++pos, ++next) {
            OSM::NodePtr a = *pos;
            OSM::NodePtr b = *next;

            len += a->distance(*b);
        }
        return len;
    }

    double length(int lower, int upper) {
        auto nds = this->nodes();
        auto begin = nds.cbegin();
        auto end = nds.cbegin();

        assert(lower <= upper && upper <= std::distance(nds.cbegin(), nds.cend()));

        std::advance(begin, lower);
        std::advance(end, upper);

        return length(begin, end);
    }

    // given point and orientation, this method will set iterators to its begin, end and the node located next to the
    // position.
    void get_pos_next_to_coordinates(double lat, double lon, double /*heading*/, int& index) {
        OSM::NodeList nds = this->nodes();
        double minDist = nds.front()->distance(lat, lon);
        auto begin = nds.begin();
        auto end = nds.end();
        index = 0;

        for (auto it = begin; it != end; ++it) {
            double dist = (*it)->distance(lat, lon);
            if (dist < minDist) {
                minDist = dist;
                index = static_cast<int>(std::distance(begin, it));
            }
        }
    }
};

class Way : public Primitive, public AbstractWay {
private:
    NodeList nodes_;

public:
    bool operator==(const Primitive& other) const override;

    std::string tag(std::string key) override {
        if (!has_tag(key)) {
            throw TagError("asked for a tag I don't have, sorry ...");
        }

        auto v = get_tag(key);
        return v;
    }

    OSM::NodeList nodes() const override {
        return this->nodes_;
    }

    bounding_box_t bb() override {
        if (nodes_.empty()) {
            throw std::runtime_error("no nodes");
        }

        double minLat, minLon, maxLat, maxLon;
        minLat = minLon = std::numeric_limits<double>::max();
        maxLat = maxLon = -minLat;

        for (const auto& n : nodes_) {
            minLat = std::min(minLat, n->lat);
            minLon = std::min(minLon, n->lon);
            maxLat = std::max(maxLat, n->lat);
            maxLon = std::max(maxLon, n->lon);
        }

        return std::make_tuple(minLat, minLon, maxLat, maxLon);
    }

    OSM::NodeList& nodes() {
        return nodes_;
    }

    using iterator = NodeList::iterator;
    using const_iterator = NodeList::const_iterator;

    explicit Way(const osmid_t& id) : Primitive(id, WAY) {
    }

    type_t type() const override {
        return WAY;
    }
};

using WayPtr = Ptr<Way>::Type;
using AbstractWayPtr = Ptr<AbstractWay>::Type;

typedef std::pair<WayPtr, std::string> WayMember;
typedef std::pair<NodePtr, std::string> NodeMember;

typedef std::pair<std::string, PrimitivePtr> Member;

using MemberList = std::list<Member>;
using PrimitiveList = List<Primitive>::Type;

class ReversedWay : public AbstractWay {
private:
    const OSM::AbstractWayPtr decorated_;

public:
    std::string tag(std::string key) override {
        return decorated_->tag(key);
    }

    explicit ReversedWay(const OSM::AbstractWayPtr& dec) : decorated_(dec) {
    }

    OSM::NodeList nodes() const override {
        OSM::NodeList nd = decorated_->nodes();
        OSM::NodeList reversed;
        reversed.insert(reversed.begin(), nd.rbegin(), nd.rend());

        return reversed;
    }
};

class PrunedWay : public AbstractWay {
private:
    AbstractWayPtr decorated_;
    OSM::NodeList::iterator first_, last_, begin_, end_;
    OSM::NodeList nds_;

public:
    PrunedWay(const AbstractWayPtr& dec, int begin, int end) : decorated_(dec), nds_(decorated_->nodes()) {
        begin_ = nds_.begin();
        end_ = nds_.end();
        first_ = last_ = begin_;

        assert(end > begin && begin >= 0 && end <= int(nds_.size()));

        std::advance(first_, begin);
        std::advance(last_, end);
    }

    OSM::NodeList nodes() const override {
        OSM::NodeList nds;
        nds.insert(nds.end(), first_, last_);
        return nds;
    }

    std::string tag(std::string /*key*/) override {
        throw std::runtime_error("not implemented");
    }
};

using PrunedWayPtr = boost::shared_ptr<PrunedWay>;

class CompositeWay : public AbstractWay {
private:
    std::list<OSM::AbstractWayPtr> ways_;

public:
    std::string tag(std::string key) override {
        /// returns a tag *only* if *all elements* (as this is a composite)
        /// have this tag (and it has the same value).

        if (this->ways_.empty()) {
            throw TagError("no members.");
        }

        std::string value = this->ways_.front()->tag(key);

        for (const auto& way : this->ways_) {
            if (way->tag(key) != value) {
                throw TagError("not all members share this tag");
            }
        }
        return value;
    }

    explicit CompositeWay(std::list<OSM::AbstractWayPtr> ways) {
        ways_.push_back(ways.front());

        if (ways.empty()) {
            throw std::runtime_error("ways empty");
        }

        ways.pop_front();

        bool foundMatch = true;

        while (!ways.empty() && foundMatch) {
            foundMatch = false;

            for (auto it = ways.begin(); it != ways.end(); ++it) {
                OSM::AbstractWayPtr way = *it;

                if (way->first() == this->last()) {
                    ways_.push_back(way);
                    foundMatch = true;
                } else if (way->last() == this->first()) {
                    ways_.push_front(way);
                    foundMatch = true;
                }

                else {
                    way = OSM::AbstractWayPtr(new OSM::ReversedWay(way));
                    if (way->first() == this->last()) {
                        ways_.push_back(way);
                        foundMatch = true;
                    } else if (way->last() == this->first()) {
                        ways_.push_front(way);
                        foundMatch = true;
                    }
                }

                if (foundMatch) {
                    ways.erase(it);
                    break;
                }
            }
        }
    }

    OSM::NodeList nodes() const override {
        OSM::NodeList nds;
        for (auto it = this->ways_.cbegin(); it != this->ways_.cend(); ++it) {
            OSM::NodeList otherNds = (*it)->nodes();
            nds.insert(nds.end(), otherNds.cbegin(), otherNds.cend());
        }


        // from c++ reference, example how to make std::vector unique
        auto it = std::unique(nds.begin(), nds.end());
        nds.resize(it - nds.begin());
        return nds;
    }

    OSM::NodePtr first() const override {
        return ways_.front()->first();
    }

    OSM::NodePtr last() const override {
        return ways_.back()->last();
    }
};

class Relation : public Primitive {
public:
    explicit Relation(const osmid_t& id) : Primitive(id, RELATION) {
    }
    bool operator==(const Primitive& other) const override;

    template <typename T>
    T get_members_with_role(const std::string& role) {
        T list;
        for (MemberList::const_iterator it = members.begin(); it != members.end(); ++it) {
            std::string currRole = it->first;
            PrimitivePtr primitive = it->second;

            if (currRole == role) {
                typename Ptr<typename T::value_type::element_type>::Type member =
                    boost::dynamic_pointer_cast<typename T::value_type::element_type>(primitive);
                if (member) {
                    list.push_back(member);
                }
            }
        }
        return list;
    }

    bounding_box_t bb() override {
        throw std::runtime_error("no sensible implementation.");
    }

    template <typename T>
    T getUniqueMember(const std::string& role) {
        OSM::PrimitiveList list = this->get_members_with_role<OSM::PrimitiveList>(role);

        if (list.size() != 1) {
            NoSuchMember nsm;
            nsm.relation_id = this->id();
            nsm.requested_member = role;
            throw nsm;
        }

        T member = boost::dynamic_pointer_cast<typename T::element_type>(list.front());
        return member;
    }

    type_t type() const override {
        return RELATION;
    }

    MemberList members;
};

std::ostream& operator<<(std::ostream& stream, const Primitive& prim);

using RelationPtr = Ptr<Relation>::Type;


using RelationList = List<Relation>::Type;

typedef std::map<PrimitiveIdentifier, PrimitivePtr> PrimitiveMap;


class OSMHelper {

public:
    static void parse_osm(const std::string& filename, PrimitiveMap& primitives);
    static void parse_osm(std::istream& stream, PrimitiveMap& primitives);
    static void write_osm(const std::string& filename, const PrimitiveMap& primitives);
    static void write_osm(std::ostream& stream, const PrimitiveMap& primitives);
    static void get_relations(const PrimitiveMap& primitives, RelationList& relations);

    template <typename T>
    static typename List<T>::Type get(const PrimitiveMap& primitives) {
        typename List<T>::Type list;

        for (auto it = primitives.cbegin(); it != primitives.cend(); ++it) {
            OSM::PrimitivePtr prim = it->second;
            auto ptr = boost::dynamic_pointer_cast<T>(prim);
            if (ptr) {
                list.push_back(ptr);
            }
        }

        return list;
    }
};

} // namespace OSM

#endif // OSMELEMENTS_H
