// File:           open_img.hpp
// Creation Date:  Friday, December 23 2011
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(OPEN_IMG_HPP)
#define OPEN_IMG_HPP

#include <vector>
#include <Magick++.h> 

// probably only works for for 16bit
template<class BaseType>
inline void open_img( std::string uri, std::vector<BaseType>& pxls, int& w, int& h, Quantum quantum = Magick::GrayQuantum )
{
  using namespace Magick; 
  using namespace std;

  Image image;
  
  image.read( uri );

  w = image.columns();
  h = image.rows();
  image.getPixels( 0, 0, w, h ); // get pixels into cache

  pxls.resize( w*h );
  
  image.writePixels( quantum, (unsigned char*)&(pxls[0]) );        
}

#endif

