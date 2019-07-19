#pragma once

#include <image_crop_coordinate_converter/types.h>
#include <opencv2/core/core.hpp>

namespace image_preproc {

cv::Mat cropAndResize(const cv::Mat& img, const ImgSize& size);

void crop(const cv::Mat&, cv::Mat&, const cv::Rect&);
void scale(const cv::Mat&, cv::Mat&, const double, const double, const uint8_t);
void resize(const cv::Mat&, cv::Mat&, const size_t , const size_t, const uint8_t);
}

