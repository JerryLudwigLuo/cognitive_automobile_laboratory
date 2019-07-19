// File:           constexpr.hpp
// Creation Date:  Saturday, May 26 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(CONSTEXPR_HPP)
#define CONSTEXPR_HPP


// workaround compatibility problems with gcc version < 4.6

#define GCC_VERSION (__GNUC__ * 10000                 \
                     + __GNUC_MINOR__ * 100           \
                     + __GNUC_PATCHLEVEL__)

#if GCC_VERSION < 40600
#define constexpr const
#endif 

#endif
