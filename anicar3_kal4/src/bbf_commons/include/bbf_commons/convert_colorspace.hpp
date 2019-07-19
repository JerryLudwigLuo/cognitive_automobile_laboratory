#ifndef CONVERT_COLORSPACE_HPP
#define CONVERT_COLORSPACE_HPP

#include <inttypes.h>
#include <iostream>

void grayscale_to_rgba( const uint8_t* gray, uint8_t* rgba, const int w, const int h, const int input_linestep, const int output_linestep, const double modulate_r = 1, const double modulate_g = 1, const double modulate_b = 1 )
{
    for( int j = 0; j<h; j++ )
    {
        for( int i=0; i<w; i++ )
        {
            // std::cout << (int)(gray[j*input_linestep+i]) << "\n";
            rgba[j*output_linestep+4*i+0] = modulate_r * gray[j*input_linestep+i];
            rgba[j*output_linestep+4*i+1] = modulate_g * gray[j*input_linestep+i];
            rgba[j*output_linestep+4*i+2] = modulate_b * gray[j*input_linestep+i];
            rgba[j*output_linestep+4*i+3] = 255;
        }
    }
}

void rgba_to_rgb( const uint8_t* rgba, uint8_t* rgb, const int w, const int h, const int input_linestep, const int output_linestep )
{
    for( int j = 0; j<h; j++ )
    {
        for( int i=0; i<w; i++ )
        {
            // std::cout << (int)(gray[j*input_linestep+i]) << "\n";
            rgb[j*output_linestep+3*i+0] = rgba[j*input_linestep+4*i+0];
            rgb[j*output_linestep+3*i+1] = rgba[j*input_linestep+4*i+1];
            rgb[j*output_linestep+3*i+2] = rgba[j*input_linestep+4*i+2];
        }
    }
}

#endif // CONVERT_COLORSPACE_HPP
