#pragma once

#include <Eigen/Dense>
#include "types.h"

namespace image_crop_coordinate_converter {
template <typename T>
using ImageCoordinate = Eigen::Matrix<T, 2, 1>;

class CoordinateConverter {
public:
    CoordinateConverter(const image_preproc::ImgSize& roiSize) : roiX{roiSize.roiX}, roiY{roiSize.roiY} {
        scaleX = static_cast<double>(roiSize.roiWidth) / roiSize.newWidth;
        scaleY = static_cast<double>(roiSize.roiHeight) / roiSize.newHeight;
    }
    template <typename T, typename U>
    ImageCoordinate<T> convertFullToCrop(const ImageCoordinate<U>& full) const {
        return ImageCoordinate<T>((full(0) - roiX) / scaleX, (full(1) - roiY) / scaleY);
    }
    template <typename T, typename U>
    ImageCoordinate<T> convertCropToFull(const ImageCoordinate<U>& cropped) const {
        return ImageCoordinate<T>(cropped(0) * scaleX + roiX, cropped(1) * scaleY + roiY);
    }

private:
    int roiX;
    int roiY;
    double scaleX; // croppedWidth / afterCropAndScaleWidth
    double scaleY;
};
}
