#pragma once

#include <opencv2/core/core.hpp>

namespace image_preproc {

class BrightnessCorrection {
public:
  static void correctGamma(const cv::InputArray& src, cv::OutputArray& dst, const double gamma);
  static double computeOptimalGamma(const cv::InputArray& src);

  static void stretchHist(const cv::InputArray& src, cv::OutputArray& dst, const double percentage);
  static void computeOptimalHist(const cv::InputArray& src, cv::OutputArray& dst);
private:
  // forbid object creation
  BrightnessCorrection();
  BrightnessCorrection(const BrightnessCorrection&);
};

} // namespace

