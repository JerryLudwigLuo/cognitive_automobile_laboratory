#ifndef HSV2RGB_HPP
#define HSV2RGB_HPP

// everything 0.0...1.0

template<class REAL>
inline void rgb2hsv( REAL r, REAL g, REAL b, REAL& h, REAL& s, REAL & v )
{
  REAL      min, max, delta;

  min = r < g ? r : g;
  min = min  < b ? min  : b;

  max = r > g ? r : g;
  max = max  > b ? max  : b;

  v = max;                                // v
  delta = max - min;
  if( max > 0.0 ) {
      s = (delta / max);                  // s
    } else {
      // r = g = b = 0                        // s = 0, v is undefined
      s = 0.0;
      h = NAN;                            // its now undefined
      return;
    }
  if( r >= max )                           // > is bogus, just keeps compilor happy
    h = ( g - b ) / delta;        // between yellow & magenta
  else
    if( g >= max )
      h = 2.0 + ( b - r ) / delta;  // between cyan & yellow
    else
      h = 4.0 + ( r - g ) / delta;  // between magenta & cyan

  h *= 60.0;                              // degrees

  if( h < 0.0 )
    h += 360.0;
  h /= 360.;
}


template<class REAL>
inline void hsv2rgb( REAL h, REAL s, REAL v, REAL& r, REAL& g, REAL& b )
{
  h *= 360.0;

  REAL      hh, p, q, t, ff;
  long        i;


  if(s <= 0.0) {       // < is bogus, just shuts up warnings
      if(isnan(h)) {   // h == NAN
          r = v;
          g = v;
          b = v;
          return;
        }
      // error - should never happen
      r = 0.0;
      g = 0.0;
      b = 0.0;
      return;
    }
  hh = h;
  if(hh >= 360.0) hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = v * (1.0 - s);
  q = v * (1.0 - (s * ff));
  t = v * (1.0 - (s * (1.0 - ff)));

  switch(i) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;

    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
    default:
      r = v;
      g = p;
      b = q;
      break;
    }
}
#endif // HSV2RGB_HPP
