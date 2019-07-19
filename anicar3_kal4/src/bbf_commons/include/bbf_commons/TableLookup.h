#pragma once

#include <vector>
#include <math.h>

enum LookupMode {
  nearest = 0,
  linear
};

// TableLookup for equidistant data
class TableLookup
{
public:
  TableLookup(double _minVal, double _maxVal, std::vector<double>& _funVals) :
    minVal(_minVal), maxVal(_maxVal), funVals(_funVals)
  {}

  double operator[](double val)
  {
    return getVal(val, linear);
  }

  double getVal(double val, LookupMode mode=linear)
  {
    if (funVals.size() == 0) {
      std::cerr << "No values in LUT!" << std::endl;
      return 0.0;
    }

    if(val <= minVal)
      return funVals.front();

    if(val >= maxVal)
      return funVals.back();

    double idxRel = (val-minVal)/(maxVal-minVal)*(funVals.size()-1);

    switch(mode)
    {
      case nearest:
      {
        int idx = round(idxRel);
        return funVals[idx];
      }
      case linear:
      {
        int idxPrev = std::max(floor(idxRel), 0);
        int idxNext = std::min(idxPrev+1, funVals.size() - 1);

        double wPrev = (idxNext-idxRel);
        double wNext = 1-wPrev;

        return (wPrev*funVals[idxPrev] + wNext*funVals[idxNext]);
      }
    }
  }

private:
  double minVal;
  double maxVal;
  std::vector<double> funVals;
};
