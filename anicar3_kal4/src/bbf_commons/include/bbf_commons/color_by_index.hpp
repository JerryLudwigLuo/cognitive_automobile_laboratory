/***************************************************************************
 *                                                                         *
 *   Author: Julius Ziegler                                                *
 *                                                                         *
 *   Copyright (C) 2014 by Atlatec UG (haftungsbeschraenkt)                *
 *                                                                         *
 *   http://atlatec.de                                                     *
 *                                                                         *
 ***************************************************************************/


#ifndef COLOR_BY_INDEX_HPP
#define COLOR_BY_INDEX_HPP

// r,g,b \in [0.0 ... 1.0]

// this one computes infinite different colors algorithmically.
inline void color_by_index( int index, double& r, double& g, double& b );

// this one chooses from a finite list. Colors look subjectively more distinct than with above
// function.
inline void color_by_index_static( int index, double& r, double& g, double& b );


namespace detail {

  int color_list[] =
  {
    0x000000,
    0x00FF00,
    0x0000FF,
    0xFF0000,
    0x01FFFE,
    0xFFA6FE,
    0xFFDB66,
    0x006401,
    0x010067,
    0x95003A,
    0x007DB5,
    0xFF00F6,
    0xFFEEE8,
    0x774D00,
    0x90FB92,
    0x0076FF,
    0xD5FF00,
    0xFF937E,
    0x6A826C,
    0xFF029D,
    0xFE8900,
    0x7A4782,
    0x7E2DD2,
    0x85A900,
    0xFF0056,
    0xA42400,
    0x00AE7E,
    0x683D3B,
    0xBDC6FF,
    0x263400,
    0xBDD393,
    0x00B917,
    0x9E008E,
    0x001544,
    0xC28C9F,
    0xFF74A3,
    0x01D0FF,
    0x004754,
    0xE56FFE,
    0x788231,
    0x0E4CA1,
    0x91D0CB,
    0xBE9970,
    0x968AE8,
    0xBB8800,
    0x43002C,
    0xDEFF74,
    0x00FFC6,
    0xFFE502,
    0x620E00,
    0x008F9C,
    0x98FF52,
    0x7544B1,
    0xB500FF,
    0x00FF78,
    0xFF6E41,
    0x005F39,
    0x6B6882,
    0x5FAD4E,
    0xA75740,
    0xA5FFD2,
    0xFFB167,
    0x009BFF,
    0xE85EBE };

  inline void get_pattern( int index, int& r, int& g, int& b ) {
    int n = (int)( std::cbrt( index ) );
    index -= (n*n*n);
    int p[3] = {n,n,n};

    // std::cout << "index n: " << index << " " << n << "\n";

    if (index == 0) {
        r = p[0];
        g = p[1];
        b = p[2];
        return;
      }

    index--;
    int v = index % 3;
    index = index / 3;

    if (index < n) {
        p[v] = index % n;
        r = p[0];
        g = p[1];
        b = p[2];
        return;;
      }

    index -= n;

    p[v      ] = index / n;
    p[++v % 3] = index % n;

    r = p[0];
    g = p[1];
    b = p[2];

    return;
  }

  int get_element( int index ) {
    int value = index - 1;
    int v = 0;
    for (int i = 0; i < 8; i++) {
        v = v | (value & 1);
        v <<= 1;
        value >>= 1;
      }
    v >>= 1;
    return v & 0xFF;
  }
}

// r,g,b \in [0.0 ... 1.0]
inline void color_by_index( int index, double& r, double& g, double& b )
{
  int ri,gi,bi;
  detail::get_pattern( index, ri, gi, bi );
  // std::cout << "rgb: " << ri << " " << gi << " " << bi << "\n";
  r = detail::get_element( ri )/255.;
  g = detail::get_element( gi )/255.;
  b = detail::get_element( bi )/255.;
}

inline void color_by_index_static( int index, double& r, double& g, double& b )
{
  index %= 64;
  int hex = detail::color_list[index];

  r = ( ( hex & 0xFF0000 ) >> 16 )/255.0;
  g = ( ( hex & 0x00FF00 ) >>  8 )/255.0;
  b = ( ( hex & 0x0000FF ) >>  0 )/255.0;

}

#endif // COLOR_BY_INDEX_HPP
