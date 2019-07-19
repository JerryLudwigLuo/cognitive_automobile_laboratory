#ifndef SERIALIZE_HPP
#define SERIALIZE_HPP

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <string>
#include <sstream>

#include "bbf_commons/serialize_tuple.hpp"

namespace BBF {
  template<class OBJECT>
  std::string to_string( const OBJECT& obj )
  {
    std::stringstream ss;
    boost::archive::binary_oarchive ar( ss );
    ar << obj;
    return ss.str();
  }

  template<class OBJECT>
  OBJECT from_string( const std::string& str )
  {
    std::stringstream ss;
    ss << str;

    boost::archive::binary_iarchive ia( ss );
    OBJECT p;
    ia >> p;

    return p;
  }

  // this version can be more convenient, because the type of
  // OBJECT can be derived by the compiler without explicit
  // qualification.
  template<class OBJECT>
  void from_string( OBJECT& obj, const std::string& str )
  {
    std::stringstream ss;
    ss << str;

    boost::archive::binary_iarchive ia( ss );
    ia >> obj;
  }
};

#endif // SERIALIZE
