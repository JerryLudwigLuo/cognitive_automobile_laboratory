// File:           random_element.hpp
// Creation Date:  Thursday, December  6 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(RANDOM_ELEMENT_HPP)
#define RANDOM_ELEMENT_HPP

template <typename It>
It random_element( It begin, It end )
{
  const unsigned long n = std::distance( begin, end );
  const unsigned long divisor = RAND_MAX / n;

  unsigned long k;
  do { k = std::rand() / divisor; } while ( k >= n );

  It result = begin;
  std::advance( result, k );
  return result;
}

#endif
