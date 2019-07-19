/***************************************************************************
 *                                                                         *
 *   Author: Julius Ziegler                                                *
 *                                                                         *
 *   Copyright (C) 2014 by Atlatec UG (haftungsbeschraenkt)                *
 *                                                                         *
 *   http://atlatec.de                                                     *
 *                                                                         *
 ***************************************************************************/

#include <iostream>     // std::cout
#include <algorithm>    // std::min_element, std::max_element

#include "verify.hpp"

#ifndef PRINT_HISTOGRAM_HPP
#define PRINT_HISTOGRAM_HPP

template<class SCALAR_ITERATOR>
void compute_and_print_histogram( SCALAR_ITERATOR begin, SCALAR_ITERATOR end, int num_bins, int output_width = 50 )
{
  SCALAR_ITERATOR min_val = std::min_element( begin, end );
  SCALAR_ITERATOR max_val = std::max_element( begin, end );

  double bin_size = (  *max_val * 1. - *min_val * 1. )/( num_bins - 1 );

  std::vector<double> histogram( num_bins, 0.0 );
  int N = 0;

  for( SCALAR_ITERATOR it = begin; it != end; it++ )
    {
      int bin = static_cast<int>(( *it - *min_val ) / bin_size);
      // std::cout << "*min_val *max_val *it bin_size bin num_bins: " << *min_val << " " << *max_val << " " << *it << " " << bin_size << " " << bin << " " << num_bins << "\n";
      verify( bin >= 0,       "!bin >= 0" );
      verify( bin < num_bins, "!bin < num_bins" );

      histogram.at(bin) += 1.0;
      N++;
    }

  //  for( int i=0; i<num_bins; i++ )
  //    std::cout << "hist: " << histogram[i]  << "\n";

  double max_bin       = *( std::max_element( histogram.begin(), histogram.end() ) );
  double perc_per_char = static_cast<int>( 100.0*max_bin/(N*output_width) + 1.);

  for( int i=0; i<num_bins; i++ )
    {
      double perc = 100.0*histogram[i]/N;
      printf( "[%8.1E %8.1E]:[%3.0f perc][%10d] ", *min_val + bin_size * i, *min_val + bin_size * (i+1), perc, static_cast<int>(histogram[i]) );
      for( int j=0; j<perc/perc_per_char; j++ )
        {
          printf( "#" );
        }
      printf( "\n" );
    }
  printf( "\n#  = %f perc\n", perc_per_char );
  printf( "[] = %f\n", bin_size );

}

#endif // PRINT_HISTOGRAM_HPP
