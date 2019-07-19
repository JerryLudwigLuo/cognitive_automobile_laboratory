// File:           convex_hull.cpp
// Creation Date:  Monday, April 23 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#include "convex_hull.hpp"

#include <cassert>
#include <deque>
#include <cmath>
#include <iostream>
#include <fstream>

void offset_polyline( const double* xs_in, const double* ys_in, double* xs_out, double* ys_out, int N, double offset )
{
  // first point is special
  {
    double x0 = xs_in[0];
    double x1 = xs_in[1];

    double y0 = ys_in[0];
    double y1 = ys_in[1];

    double xn = -( y1 - y0 );
    double yn = x1 - x0;

    double norm = hypot( xn, yn );

    yn /= norm;
    xn /= norm;

    double dx = xn * offset;
    double dy = yn * offset;

    xs_out[0] = ( x0 + dx );
    ys_out[0] = ( y0 + dy );

    // xs_out.push_back( x0 + xn * offset );
    // ys_out.push_back( y0 + yn * offset );
  }

  for( int i=1; i<N-1; i++ )
    {
      double x0 = xs_in[i-1];
      double x1 = xs_in[i];
      double x2 = xs_in[i+1];

      double y0 = ys_in[i-1];
      double y1 = ys_in[i];
      double y2 = ys_in[i+1];

      double xn = -( y2 - y0);
      double yn = x2 - x0;

      double norm = hypot( xn, yn );
      yn /= norm;
      xn /= norm;

      xs_out[i] = ( x1 + xn * offset );
      ys_out[i] = ( y1 + yn * offset );


    }

  // last point is special
  {
    double x0 = xs_in[N-2];
    double x1 = xs_in[N-1];

    double y0 = ys_in[N-2];
    double y1 = ys_in[N-1];

    double xn = -( y1 - y0);
    double yn = x1 - x0;

    double norm = hypot( xn, yn );

    yn /= norm;
    xn /= norm;

    double dx = xn * offset;
    double dy = yn * offset;

    // xs_out.push_back( x1 + xn * offset );
    // ys_out.push_back( y1 + yn * offset );

    xs_out[N-1] = ( x1 + dx );
    ys_out[N-1] = ( y1 + dy );
  }
}

void offset_polyline( const std::vector<double>& xs_in, const std::vector<double>& ys_in, std::vector<double>& xs_out, std::vector<double>& ys_out, double offset )
{
  assert( xs_in.size() == ys_in.size() );
  xs_out.clear();
  ys_out.clear();

  int N = xs_in.size();

  xs_out.resize( N );
  ys_out.resize( N );

  offset_polyline( &(xs_in[0]), &(ys_in[0]), &(xs_out[0]), &(ys_out[0]), N, offset );
  assert( xs_out.size() == ys_out.size() );
}

// pre: xs_in sorted 
void filter_max_m( const std::vector<double>& xs_in, const std::vector<double>& ys_in, std::vector<double>& xs_out, std::vector<double>& ys_out, double max_m )
{
  const unsigned int N = xs_in.size();
  std::vector<bool> dominant( N, true );

  for( unsigned int i = 0; i<N; i++ )
    {
      double x = xs_in[i];
      double y = ys_in[i];
      double b = y-max_m*x;

      for( unsigned int j = 0; j<i; j++ )
        {
          double xt = xs_in[j];
          double yt = ys_in[j];

          if( max_m*xt + b >= yt )
            dominant[j] = false;
        }

      b = y+max_m*x;
      for( unsigned int j = i+1; j<N; j++ )
        {
          double xt = xs_in[j];
          double yt = ys_in[j];
          if( -max_m*xt + b >= yt )
            dominant[j] = false;
        }
    }

  std::deque<double> xs_out_dq, ys_out_dq;

  for( unsigned int i = 0; i<N; i++ )
    {
      if( dominant[i] )
        {
          xs_out_dq.push_back( xs_in[i] );
          ys_out_dq.push_back( ys_in[i] );
        }
    }

  // catch the cases were there is already a point at y = 0 (at front and back)
  // (happens when appyling this function a second time, as done in the join_overlapping(...) function
  // of StixelConstraintProcessor)
  if( ys_out_dq.front() != 0. )
    {
      xs_out_dq.push_front( xs_out_dq.front()-ys_out_dq.front()/max_m );
      ys_out_dq.push_front( 0. );
    }
  if( ys_out_dq.back() != 0. )
    {
      xs_out_dq.push_back( xs_out_dq.back()+ys_out_dq.back()/max_m );
      ys_out_dq.push_back( 0. );
    }
  xs_out.clear();
  ys_out.clear();

  xs_out.insert( xs_out.begin(), xs_out_dq.begin(), xs_out_dq.end() );
  ys_out.insert( ys_out.begin(), ys_out_dq.begin(), ys_out_dq.end() );
}

// pre: xs_in sorted 
void convex_hull( const std::vector<double>& xs_in, const std::vector<double>& ys_in, std::vector<double>& xs_out, std::vector<double>& ys_out )
{
  // in some corner cases, there can be doublettes in xs_in. This is a not very sophisticated but efficient way to get rid of them:
  std::vector<double> xs_in_ = xs_in;
  for( size_t i=1; i<xs_in.size(); i++ )
    {
      const double SMALL_VALUE = 0.01;

      while( xs_in_[i] - xs_in_[i-1] <= SMALL_VALUE )
        xs_in_[i] += SMALL_VALUE;
    }
  
  std::cout << "convex_hull: xs_in_.size() == " << xs_in_.size() << "\n";
  assert( xs_in_.size() > 0 );
  assert( xs_in_.size() == ys_in.size() );

  size_t I = 0;
  xs_out.clear();
  ys_out.clear();

  xs_out.push_back( xs_in_[0] );
  ys_out.push_back( ys_in[0] );

  const size_t N = xs_in_.size();
  while( I != N - 1 )
    {
      if( I > N - 1 )
        {
          // something bad has happened
          std::ofstream dump( "/tmp/ch.txt" );

          for( size_t i = 0; i<N; i++ )
            dump << xs_in_[i] << " " << ys_in[i] << "\n";
          
          std::ofstream dump2( "/tmp/h.txt" );
          for( size_t i = 0; i<xs_out.size(); i++ )
            dump2 << xs_out[i] << " " << ys_out[i] << "\n";

          std::cout << "convex hull error, dumped to /tmp/ch.txt\n";
          dump.close();
          dump2.close();
          exit(1);
        }

      std::cout << "I N: " << I << " " << N << "\n";
      double x0 = xs_in_[I];
      double y0 = ys_in[I];
      for( size_t i = I+1; I < N; i++ )
        {
          double x1 = xs_in_[i];
          double y1 = ys_in[i];
          double m = (y1-y0)/(x1-x0);
          double b=y0-m*x0;
          bool accept = true;
          for( size_t j = 0; j<N; j++ )
            {
              double xt = xs_in_[j];
              double yt = ys_in[j];

              if( j != i and j != I )
                {
                  if( m*xt + b < yt )
                    {
                      accept = false;
                      break;
                    }
                }
            }
          if( accept )
            {
              I = i;
              xs_out.push_back( xs_in_[I] );
              ys_out.push_back( ys_in[I] );
              break;
            }
        }
    }
}

