#pragma once

#include "Polygon.hpp"

#include <boost/serialization/vector.hpp>
#include <jsoncpp/json/json.h>

namespace boost {
namespace serialization {

template<class Archive>
void serialize( Archive & ar, BBF::Polygon& poly, const unsigned int version )
{
    ar & poly.xs;
    ar & poly.ys;
}

} // namespace serialization
} // namespace boost

#ifdef USE_JSON

namespace BBF
{

inline
Json::Value to_json(const BBF::Polygon& poly)
{
    Json::Value json_poly;
    for( int i = 0; i < poly.xs.size(); ++i )
    {
        Json::Value pt;
        double x, y;
        std::tie(x, y) = poly.at(i);
        pt.append(x);
        pt.append(y);
        json_poly["points"].append(pt);
    }

    json_poly["label"] = poly.label;
    return json_poly;
}

}

#endif

