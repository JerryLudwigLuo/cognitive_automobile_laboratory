#pragma once

#include <boost/date_time.hpp>

/// taken from

inline
boost::posix_time::ptime parse_posix_time(const std::string& format, const std::string& localDate)
{
#warning "Here http://stackoverflow.com/questions/11121454/c-why-is-my-date-parsing-not-threadsafe they say that this function is NOT threadsafe."
    std::istringstream is(localDate);
    is.imbue(std::locale(is.getloc(), new boost::local_time::local_time_input_facet(format.c_str())));
    boost::posix_time::ptime pt;
    is >> pt;

    if (pt == boost::posix_time::ptime())
    {
        throw std::runtime_error("Parse error");
    }

    return pt;
}
