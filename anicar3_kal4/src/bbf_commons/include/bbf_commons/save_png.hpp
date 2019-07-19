// File:           save_png.hpp
// Creation Date:  Monday, January 16 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#include <string>

#include <iostream>

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_io.hpp>
#include <boost/bind.hpp>

// Can create 16 bit pngs.
// Must link libpng (-lpng).

// PixelType: from boost::gil, e.g.

// boost::gil::gray8_pixel_t
// boost::gil::gray16_pixel_t
// boost::gil::rgb8_pixel_t
// boost::gil::gray32f_pixel_t // <- floating point
// ...
// (but see below for 64 bit (double) floating point pixel type)



namespace MRT {
  // typedef boost::gil::pixel< double, boost::gil::gray_layout_t > gray64f_pixel_t; 

  template<class PixelType>
  void save_png( const void* data, int w, int h, std::string filename )
  {
    using namespace boost::gil;
    
    typename type_from_x_iterator<const PixelType*>::view_t view = interleaved_view( w, h, (const PixelType*)data, w*sizeof( PixelType ) );
    png_write_view( filename.c_str(), view );
  }

  template<class Real>
  void save_png_real( const Real* data, int w, int h, std::string filename, double black = 0.0, double white = 255.0 )
  {
    uint8_t data8[w*h];
    int N = w*h;
    for( int i=0; i<N; i++ )
      {
        // std::cout << data[i] << " ";
        double v     = 255*scale_and_crop( data[i], black, white );
        // std::cout << v << " ";
        uint8_t v8  = (uint8_t)(v);
        // std::cout << (int)v8 << "\n";
        data8[i]    = v8;
      }
    save_png<boost::gil::gray8_pixel_t>( data8, w, h, filename );
  }

  template<class SourcePixelType, class TargetFilePixelType>
  void save_png_generic( const void* data, int w, int h, std::string filename )
  {
    using namespace boost::gil;
    
    typename type_from_x_iterator< const SourcePixelType* >::view_t source_view = interleaved_view( w, h, ( const SourcePixelType* )data, w*sizeof( SourcePixelType ) );
    
    image<TargetFilePixelType, false> converted_image( source_view.dimensions() );
    copy_and_convert_pixels( source_view, view( converted_image ) );

    png_write_view( filename.c_str(), view( converted_image ) );
  }

  inline double scale_and_crop( double v, double black, double white )
  {
    double scaled = ( v - black ) / ( white - black );
    if( scaled < 0 ) scaled = 0;
    if( scaled > 1 ) scaled = 1;
    return scaled;
  }

  template<class SourcePixelType, class TargetFilePixelType>
  void save_png_generic( const void* data, int w, int h, std::string filename, double black, double white )
  {
    using namespace boost::gil;
    
    typename type_from_x_iterator< const SourcePixelType* >::view_t source_view = interleaved_view( w, h, ( const SourcePixelType* )data, w*sizeof( SourcePixelType ) );
    
    image<SourcePixelType, false> scaled_image( source_view.dimensions() );
 
    for (int y=0; y<source_view.height(); ++y)
      for (int x=1; x<source_view.width()-1; ++x)
        {
          view( scaled_image )(x,y) = scale_and_crop( source_view(x,y)[0], black, white );
          // std::cout << scale_and_crop( source_view(x,y)[0], black, white ) << "\n";
        }

    image<TargetFilePixelType, false> converted_image( source_view.dimensions() );
    copy_and_convert_pixels( view( scaled_image ), view( converted_image ) );

    png_write_view( filename.c_str(), view( converted_image ) );
  }
}

// a change
