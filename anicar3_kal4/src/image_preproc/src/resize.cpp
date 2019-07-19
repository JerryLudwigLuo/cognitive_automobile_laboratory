#include "resize.h"
#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"

cv::Mat image_preproc::cropAndResize(const cv::Mat& img, const ImgSize& size) {
    cv::Rect roi(size.roiX, size.roiY, size.roiWidth, size.roiHeight);
    cv::Mat cropped = img(roi);
    if (size.newHeight != 0 && size.newWidth != 0) {
        cv::Mat resized;
        cv::resize(cropped, resized, cv::Size(size.newWidth, size.newHeight));
        return resized;
    } else {
        return cropped;
    }
}

void image_preproc::crop(const cv::Mat& src, cv::Mat& dst, const cv::Rect& roi) {
    //std::cout << "Cropping - u: " << roi.x << " - " << roi.width <<
    //                     " | v: " << roi.y << " - " << roi.height << std::endl;
    dst = src(roi);
}

void image_preproc::scale(const cv::Mat& src, cv::Mat& dst, const double scaling_u, const double scaling_v, const uint8_t interpolation) {
    //std::cout << "Scaling - u: " << scaling_u << " | v: " << scaling_v << std::endl;
    cv::resize(src, dst, cv::Size(0, 0), scaling_u, scaling_v, interpolation);
}

void image_preproc::resize(const cv::Mat& src, cv::Mat& dst, const size_t resize_u, const size_t resize_v, const uint8_t interpolation) {
    //std::cout << "Resize - u: " << resize_u << " | v: " << resize_v << std::endl;
    cv::resize(src, dst, cv::Size(resize_u, resize_v), 0, 0, interpolation);
}