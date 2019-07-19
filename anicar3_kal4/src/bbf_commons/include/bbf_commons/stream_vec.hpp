/***************************************************************************
 *                                                                         *
 *   Author: Julius Ziegler                                                *
 *                                                                         *
 *   Copyright (C) 2014 by Atlatec UG (haftungsbeschraenkt)                *
 *                                                                         *
 *   http://atlatec.de                                                     *
 *                                                                         *
 ***************************************************************************/


#ifndef STREAM_VEC_HPP
#define STREAM_VEC_HPP

#include <iostream>

template<class T>
void write_vec( std::ostream &output, const T* vec, int len )
{
  for( int i=0; i<len; i++ )
    {
      output << vec[i] << " ";
    }
  output << "\n";
 }

/*
template<class ITERATOR>
void write_vec( std::ostream &output, ITERATOR begin, const ITERATOR end )
{
  for( ; begin != end; begin++ )
    output << *begin << " ";
  output << "\n";

}
*/

template<class T>
void read_vec( std::istream &input, T* vec, int len )
{
  for( int i=0; i<len; i++ )
    {
      input >> vec[i];
    }
}

/*
template<class ITERATOR>
void read_vec( std::istream &input, ITERATOR begin, const ITERATOR end )
{
  for( ; begin != end; begin++ )
    input >> *begin;
}
*/

#endif // STREAM_VEC_HPP
