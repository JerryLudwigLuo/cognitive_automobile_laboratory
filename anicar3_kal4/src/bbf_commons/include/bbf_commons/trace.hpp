// File:           trace.hpp
// Creation Date:  Wednesday, April 27 2011
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(TRACE_HPP)
#define TRACE_HPP


namespace MRT_trace {
  const unsigned int critical        = 1;
  const unsigned int warning         = 2;
  const unsigned int debug           = 4;
  
  const unsigned int warning_plus    = critical|2;
  const unsigned int debug_plus      = critical|warning|4;

  const unsigned int gui                 = 8;
  const unsigned int initialisation      = 16;
  const unsigned int rtdb                = 32;
  const unsigned int solver              = 64;
  const unsigned int solver_verbose      = 128;
  const unsigned int controller          = 256;

  const unsigned int all                 = 0xFFFFFFFF;
  
}

#ifndef TRACE_LEVEL
#define TRACE_LEVEL critical
#endif

//#define TRACE_LEVEL all

#define TRACE_STREAM_TAGGED( trace_level, msg ) if( true ) { using namespace MRT_trace; if( trace_level & TRACE_LEVEL ) { std::cout << "TRACE: " << __FILE__ << ": " << __LINE__ << ": " << __func__ << ": " << msg << std::endl; } }
#define TRACE_STREAM( trace_level, msg ) if( true ) { using namespace MRT_trace; if( trace_level & TRACE_LEVEL ) { std::cout << msg << std::endl; } }

#endif
