#ifndef sample_gauniform_hpp__
#define sample_gauniform_hpp__

#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution

using boost::math::normal; // typedef provides default type is double.

inline double sample_gauss()
{
  // select random number generator
  static boost::mt19937 rng;
   
  // select desired probability distribution
  boost::normal_distribution<double> n;
  
  // bind random number generator to distribution, forming a function
  boost::variate_generator< boost::mt19937&, boost::normal_distribution<double> >  normal_sampler( rng, n );
  
  // sample from the distribution
  return normal_sampler();
}

#endif
