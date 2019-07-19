
/*
 * Copyright 2012. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Henning Lategahn (henning.latgahn@kit.edu)
 *  and others
 */

#pragma once

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include "generic_logger/generic_logger.hpp"
#include "stringHelper.h"
#include <boost/optional.hpp>


/*@class cOpenCvHelper
 *
 * @par Purpose
 * helper functions to create iplimages etc. .
 *
 * @todo: write more comments
 *
 */
class cOpenCvHelper
{

    public: /* public classes/enums/types etc... */


    template<typename T>
    struct LocalizedPatch {

        virtual ~LocalizedPatch() = default;
        cv::Mat patch_;
        Eigen::Matrix<T, 2, 1> posBottomLeft_{ Eigen::Matrix<T, 2, 1>::Zero() };
        Eigen::Matrix<T, 2, 1> size_{ Eigen::Matrix<T, 2, 1>::Zero() };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

      // Atlatec CD colors ;-)
      
      static const cv::Scalar green()
      {
        return cv::Scalar(227, 237, 224);
      }

      static const cv::Scalar red()
      {
        return cv::Scalar(74, 54, 231);
      }

      static const cv::Scalar gray()
      {
        return cv::Scalar(69, 54, 52);
      }

      /**
       * @brief converts any 2d array of type T into a color image much like Matlabs imagesc
       * @return none
       * @param none
       */
      template <typename T >
        static cv::Mat arrayToColorMap( T * array, int width, int height, bool bIgnoreZero = true )
        {
          uint8_t colormap[256][3] = {
#include "color4.map"
          };

          cv::Mat M(height, width, CV_8UC3 );
          M = cv::Scalar(0,0,0);

          // find min and max of array
          double minVal = array[0];
          double maxVal = array[0];
          for (int i = 0; i < width * height; ++i)
          {
            if ( bIgnoreZero && array[i] == 0)
            {
              continue;
            }
            minVal = std::min( (double)array[i], minVal );
            maxVal = std::max( (double)array[i], maxVal );
          }

#ifdef DEBUG
          std::cout  << "Colormap:" << "\n";
          std::cout  << "Min: " << minVal << "\n";
          std::cout  << "Max: " << maxVal << "\n";
#endif

          int range = (maxVal - minVal);
          if (range == 0)
          {
            range = 1;
          }

          for (int v = 0; v < height; ++v)
          {
            for (int u = 0; u < width; ++u)
            {
              double value = (double) array[ v * width + u ];
              if ( bIgnoreZero && value == 0)
              {
                continue;
              }

              int i = (value - minVal) / (range) * 255;

              M.at<cv::Vec3b>(v,u)[0] = colormap[i][0];
              M.at<cv::Vec3b>(v,u)[1] = colormap[i][1];
              M.at<cv::Vec3b>(v,u)[2] = colormap[i][2];
            }
          }

          return M;
        }


      /** see matlab linspace */
      static vector<double> linspace( double minV, double maxV, int num )
      {
        vector<double> rv;

        double range = maxV - minV;
        double step = range/num;

        for (int i = 0; i < num+1; ++i)
        {
          rv.push_back( ((double)i) * step + minV );
        }

        return rv;
      }

      static double mean( vector<double> vecValue )
      {
        double sum = 0;
        for (auto v:vecValue)
        {
          sum += v;
        }
        return sum/vecValue.size();
      }

      template <typename T>
      static double sum( vector<T> vecValue )
      {
        double sum = 0;
        for (auto v:vecValue)
        {
          sum += v;
        }
        return sum;
      }

      static vector<double> toHistogram( vector<double> vecValues,  vector<double> vecBinCenters )
      {
        vector<double> vecBins( vecBinCenters.size(), 0 );
        for ( auto v:vecValues )
        {

          double fMinDist = fabs(vecBinCenters[0] - v);
          int iMin = 0;
          for (size_t i = 0; i < vecBinCenters.size(); ++i)
          {
            if ( fabs(vecBinCenters[i] - v) < fMinDist )
            {
              fMinDist = fabs(vecBinCenters[i] - v);
              iMin = i;
            }
          }
          vecBins[iMin] += 1;
        }

        return vecBins;
      }


      /**
       * @brief compute a histogram from values vecValues
       * @return none
       * @param
       */
      template <typename T>
        static cv::Mat histogramFromVector( vector<T> vecValues, float width, float height, vector<T> vecBinCenters )
        {

          cv::Mat M(height, width, CV_8UC3, cv::Scalar(0,0,0) );

          if (vecValues.size() == 0)
          {
            return M;
          }

          vector<int> vecBins( vecBinCenters.size(), 0 );
          float iMaxBin = 0;
          float sum = 0;
          for ( float v:vecValues )
          {
            sum += v;
            float fMinDist = fabs(vecBinCenters[0] - v);
            int iMin = 0;
            for (size_t i = 0; i < vecBinCenters.size(); ++i)
            {
              if ( fabs(vecBinCenters[i] - v) < fMinDist )
              {
                fMinDist = fabs(vecBinCenters[i] - v);
                iMin = i;
              }
            }
            vecBins[iMin]++;

            if ( vecBins[iMin] > iMaxBin )
            {
              iMaxBin = vecBins[iMin];
            }

          }
          sum /= vecValues.size();

          // draw
          float margin = 20;
          width -= 2*margin;
          height -= 3*margin;
          float binWidth = width /  vecBinCenters.size();
          for (size_t i = 0; i < vecBins.size(); ++i)
          {
            rectangle( M, cv::Point( margin + binWidth * i, height + margin), cv::Point( margin + binWidth * (i+1) - 3, height + margin - ((float)(vecBins[i]) * height / iMaxBin ) ), cv::Scalar(200,140,0), CV_FILLED );
            char buf[100];
            sprintf( buf, "%2.2f", vecBinCenters[i]);
            putText( M, buf, cv::Point( margin + binWidth * i + binWidth / 2 - 20, height + margin * 2), cv::FONT_HERSHEY_COMPLEX, .6, cv::Scalar(0,200,200));
          }

          char buf[100];
          sprintf( buf, "mean: %2.2f", sum);
          putText( M, buf, cv::Point( width - margin - 100, margin * 2), cv::FONT_HERSHEY_COMPLEX, .6, cv::Scalar(0,200,200));
          sprintf( buf, "sum: %i", (signed) vecValues.size());
          putText( M, buf, cv::Point( width - margin - 100, margin * 3), cv::FONT_HERSHEY_COMPLEX, .6, cv::Scalar(0,200,200));
          return M;

        }

      /**
       * @brief append one opencv image to the bottom of the other
       * @return none
       * @param
       */
      static cv::Mat appendToBottom( cv::Mat Mtop, cv::Mat Mbottom )
      {

        int width = std::max( Mtop.cols, Mbottom.cols );
        int height = Mtop.rows + Mbottom.rows;

        int flag = 0;
        if (Mtop.channels() == 1)
        {
          flag = CV_8UC1;
        }
        else
        {
          flag = CV_8UC3;
        }
        cv::Mat M = cv::Mat::zeros(height, width, flag );

        cv::Mat roi1 = M(cv::Rect(0,0,Mtop.cols, Mtop.rows));
        Mtop.copyTo(roi1);

        cv::Mat roi2 = M(cv::Rect(0,Mtop.rows, Mbottom.cols, Mbottom.rows));
        Mbottom.copyTo(roi2);

        return M;
      }

      static cv::Mat appendToRight( cv::Mat Mleft, cv::Mat Mright )
      {

        int width = Mleft.cols + Mright.cols ;
        int height = std::max(Mleft.rows, Mright.rows);

        int flag = 0;
        if (Mleft.channels() == 1)
        {
          flag = CV_8UC1;
        }
        else
        {
          flag = CV_8UC3;
        }
        cv::Mat M = cv::Mat::zeros(height, width, flag );

        cv::Mat roi1 = M(cv::Rect(0,0,Mleft.cols, Mleft.rows));
        Mleft.copyTo(roi1);

        cv::Mat roi2 = M(cv::Rect(Mleft.cols, 0, Mright.cols, Mright.rows));
        Mright.copyTo(roi2);

        return M;
      }


      static cv::Mat gray2color( cv::Mat m )
      {
        cv::Mat M(m.rows, m.cols, CV_8UC3 );

        for (int v = 0; v < m.rows; ++v)
        {
          for (int u = 0; u < m.cols; ++u)
          {
            M.at<cv::Vec3b>(v,u)[0] = m.at<uint8_t>(v,u);
            M.at<cv::Vec3b>(v,u)[1] = m.at<uint8_t>(v,u);
            M.at<cv::Vec3b>(v,u)[2] = m.at<uint8_t>(v,u);
          }
        }
        return M;

      }

      static bool readImageFromFile(const std::string &fName, cv::Mat &img, const int imageType) {
          img = cv::imread( fName.c_str(), imageType );
          return img.data && img.rows >= 0 && img.cols >= 0;
      }

      static bool readGreyScaleImageFromFile(const std::string &fName, cv::Mat &img) {
          return readImageFromFile(fName, img, CV_LOAD_IMAGE_GRAYSCALE);
      }

      static bool readGreyScaleImageFromFile(cv::Mat &img, const std::string &path, const int &imgNumber, const std::string &imgFileFormat = "png") {
          return readGreyScaleImageFromFile(path +  "/" + stringHelper::num2str(imgNumber) + "." + imgFileFormat, img);
      }

      static cv::Mat readImageFromFile(const std::string &path, const int imageType) {
          cv::Mat img;
          if( !readImageFromFile(path, img, imageType) ) {
              ERROR_STREAM("Error: Couldnt get image from file " << path);
              throw std::runtime_error("Couldnt get image from file");
          }
          return img;
      }

      static cv::Mat readGreyScaleImageFromFile(const std::string &path) {
          cv::Mat img;
          if( !readGreyScaleImageFromFile(path, img) ) {
              ERROR_STREAM("Error: Couldnt get image from file " << path);
              throw std::runtime_error("Couldnt get image from file");
          }
          return img;
      }

      static cv::Mat readGreyScaleImageFromFile(const std::string &path, const int &imgNumber, const std::string &imgFileFormat = "png") {
          return readGreyScaleImageFromFile(path +  "/" + stringHelper::num2str(imgNumber) + "." + imgFileFormat);
      }


      template<typename T>
      inline static void getImagePatch(
              const cv::Mat& img,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& ip,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& sz,
              cv::Mat& patch,
              Eigen::Ref<Eigen::Matrix<T, 2, 1> > positionBottomLeft,
              Eigen::Ref<Eigen::Matrix<T, 2, 1> > szSetted) {

          positionBottomLeft = ip.template cast<double>() - sz / 2;
          positionBottomLeft[0] = std::max(positionBottomLeft[0], 0.);
          positionBottomLeft[1] = std::max(positionBottomLeft[1], 0.);
          szSetted = sz;
          if(positionBottomLeft[0] + szSetted[0] > img.cols) szSetted[0] = img.cols - positionBottomLeft[0];
          if(positionBottomLeft[1] + szSetted[1] > img.rows) szSetted[1] = img.rows - positionBottomLeft[1];

          img( cv::Rect(positionBottomLeft[0], positionBottomLeft[1], szSetted[0], szSetted[1]) ).copyTo(patch);
      }

      template<typename T>
      inline static void getImagePatch(
              const cv::Mat& img,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& ip,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& sz,
              LocalizedPatch<T>& patch) {
          getImagePatch<T>(img, ip, sz, patch.patch_, patch.posBottomLeft_, patch.size_);
      }

      template<typename T>
      inline static void getImagePatches(
              const cv::Mat& img,
              const std::vector< Eigen::Matrix<T, 2, 1>  >& ips,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& sz,
              std::vector< LocalizedPatch<T> >& patches) {
          patches.resize(ips.size());
          for(size_t k = 0; k < ips.size(); k++) getImagePatch<T>(img, ips[k], sz, patches[k]);
      }

      template<typename T>
      inline static cv::Mat getImagePatch(
              const cv::Mat& img,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& ip,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& sz) {
          Eigen::Vector2d pos, range;
          cv::Mat patch;
          getImagePatch<T>(img, ip, sz, patch, pos, range);
          return patch;
      }

      inline static boost::optional<cv::Mat> getImagePatchFixSize(
              const cv::Mat& img,
              const int u,
              const int v,
              const int w,
              const int h) {
          const auto tl_u = u - w / 2;
          const auto tl_v = v - h / 2;
          if(  (tl_u) < 0
            || (tl_v) < 0
            || (tl_u + w) > img.cols
            || (tl_v + h) > img.rows ) return boost::optional<cv::Mat>();
          return boost::make_optional(img( cv::Rect(tl_u, tl_v, w, h) ));
      }

      template<typename T>
      inline static boost::optional<cv::Mat> getImagePatchFixSize(
              const cv::Mat& img,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& ip,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& sz) {
          const Eigen::Vector2i ipInt = ip.template cast<int>();
          const Eigen::Vector2i szInt = sz.template cast<int>();
          return getImagePatchFixSize(img, ipInt[0], ipInt[1], szInt[0], szInt[1]);
      }

      template<typename T>
      inline static std::vector<boost::optional<cv::Mat> > getImagePatchesFixSize(
              const cv::Mat& img,
              const std::vector< const Eigen::Matrix<T, 2, 1> >& ips,
              const Eigen::Ref<const Eigen::Matrix<T, 2, 1> >& sz) {
          std::vector<boost::optional<cv::Mat> > patches(ips.size());
          for(size_t k = 0; k < ips.size(); k++) patches[k] = getImagePatchFixSize<T>(img, ips[k], sz);
          return patches;
      }

      static void HSVtoRGB(float *r, float *g, float *b, float h, float s, float v)
      {
          int i;
          float f, p, q, t;
          if(s == 0)
          {
              // achromatic (grey)
              *r = *g = *b = v;
              return;
          }
          h /= 60;			// sector 0 to 5
          i = floor(h);
          f = h - i;			// factorial part of h
          p = v * (1 - s);
          q = v * (1 - s * f);
          t = v * (1 - s * (1 - f));
          switch(i)
          {
              case 0:
                  *r = v;
                  *g = t;
                  *b = p;
                  break;
              case 1:
                  *r = q;
                  *g = v;
                  *b = p;
                  break;
              case 2:
                  *r = p;
                  *g = v;
                  *b = t;
                  break;
              case 3:
                  *r = p;
                  *g = q;
                  *b = v;
                  break;
              case 4:
                  *r = t;
                  *g = p;
                  *b = v;
                  break;
              default:		// case 5:
                  *r = v;
                  *g = p;
                  *b = q;
                  break;
          }
      }

      static void convertHSV2RGB(float intensity, float angleBegin, float angleEnd, float& r, float& g, float& b)
      {

          float valMargin = angleEnd - angleBegin;

          float s = 1.0f;
          float v = 1.0f;
          float h = ((1.0f - intensity) * valMargin) + angleBegin;

          HSVtoRGB(&r, &g, &b, h, s, v);
      }

      static void colorDist(const double& dist, const double& maxDist, float &r, float &g, float &b)
      {

          float startAng = 0.0f;
          float endAng = 270.0f;
          float val = dist / maxDist;
          float c = std::max(0.0f, std::min(1.0f, val));
          convertHSV2RGB(c, startAng, endAng, r, g, b);
          r *= 255.;
          g *= 255.;
          b *= 255.;
      }

      static inline double interpolate(
          const double& val, const double& y0, const double& x0, const double& y1, const double& x1) {
          return ((val - x0) * (y1 - y0) / (x1 - x0) + y0) * 255;
      }
      static double base(const double& val) {
          if (val <= -0.75)
              return 0;
          else if (val <= -0.25)
              return interpolate(val, 0.0, -0.75, 1.0, -0.25);
          else if (val <= 0.25)
              return 255;
          else if (val <= 0.75)
              return interpolate(val, 1.0, 0.25, 0.0, 0.75);
          else
              return 0.0;
      }
      static inline double red(const double& gray) {
          return base(gray - 0.5);
      }
      static inline double green(const double& gray) {
          return base(gray);
      }
      static inline double blue(const double& gray) {
          return base(gray + 0.5);
      }

      static void colorDistJet(
              const double& v,
              const double& min,
              const double& max,
              double &r, double &g, double &b)
      {
          const auto vScaled = (v - min) / (max - min);
          r = red(vScaled);
          g = green(vScaled);
          b = blue(vScaled);
      }

      static void scalarToRgb(const double& v, const double& min, const double& max, double& r, double& g, double& b) {

          const auto ratio = 2. * (v-min) / (max - min);
          r = std::max<double>(0., ratio - 1);
          b = std::max<double>(0., 1 - ratio);
          g = 1 - b - r;
      }

      static void scalarToRgb(const double& v, const double& min, const double& max, uint8_t& r, uint8_t& g, uint8_t& b) {

         double rd, gd, bd;
         scalarToRgb(v, max, min, rd, gd, bd);
         r = static_cast<uint8_t>( rd * 255. );
         g = static_cast<uint8_t>( gd * 255. );
         b = static_cast<uint8_t>( bd * 255. );
      }
    };
