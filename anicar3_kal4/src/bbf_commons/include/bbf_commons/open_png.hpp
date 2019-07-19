// File:           save_png.hpp
// Creation Date:  Monday, January 16 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#include <string>
#include <vector>

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_io.hpp>

// Can create 16 bit pngs.
// Must link libpng (-lpng).

// PixelType: from boost::gil, e.g.

// boost::gil::gray8_pixel_t
// boost::gil::gray16_pixel_t
// boost::gil::rgb8_pixel_t
// ...

namespace MRT {
  template<class PixelType>
  void open_png( std::string filename, std::vector<unsigned char>& image_data, int& w, int& h, int& width_byte_step )
  {
    using namespace boost::gil;
    
    image<PixelType, false> runtime_image;

    png_read_image( filename, runtime_image );

    int bytes = runtime_image.width() * runtime_image.height() * sizeof( PixelType );
    image_data.resize( bytes );
    
    memcpy( &(image_data[0]), (void*)( &( *( view( runtime_image ).begin() ) ) ), bytes );

    w = runtime_image.width();
    h = runtime_image.height();
    width_byte_step = sizeof( PixelType ) * w;
  }
}


