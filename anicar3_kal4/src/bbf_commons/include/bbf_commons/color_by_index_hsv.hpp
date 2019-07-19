/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */
#ifndef COLORBYINDEX_HPP
#define COLORBYINDEX_HPP

#include <opencv2/opencv.hpp>

namespace ColorByIndex{
inline cv::Scalar hsv2bgr(cv::Scalar ColorHSV)
{
    cv::Mat EllipseColor(1,1,CV_8UC3,ColorHSV);
    cv::Mat NewEllipseColor(1,1,CV_8UC3);
    cv::cvtColor(EllipseColor,NewEllipseColor,CV_HSV2BGR);
    return cv::Scalar(NewEllipseColor.at<cv::Vec3b>(0,0));
}
inline cv::Scalar get_color(uint32_t ID,int NumColors=6)
{
    if(ID==0)
    {
        return cv::Scalar(123,22,234);
    }
    else{
        uint32_t ModID = ID-1;
        ModID = ModID % NumColors;

        int dH=360/NumColors;
        int S=200;
        int V=200;

        return hsv2bgr(cv::Scalar(ModID*dH,S,V));
    }
}

}
#endif // COLORBYINDEX_HPP
