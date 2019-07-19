// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values are almost equal (4
// ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values are almost equal (4
// ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between val1 and val2
// doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "gtest/gtest.h"

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(TestGroupName, TestName) {
// TODO: Add your test code here
//}

#include "resize.h"

#include <opencv2/highgui/highgui.hpp>

using namespace image_preproc;

TEST(Resize, Crop) {
    cv::Mat img = cv::imread("test_data/image.png", CV_LOAD_IMAGE_GRAYSCALE);
    ImgSize sz({8, 8, 8, 8, 0, 0});
    cv::Mat newImg = cropAndResize(img, sz);
    cv::Size expected(8, 8);
    EXPECT_EQ(newImg.size(), expected);
    EXPECT_EQ(newImg.at<uint8_t>(0, 0), 0);
    EXPECT_EQ(newImg.at<uint8_t>(4, 7), 255);
}

TEST(Resize, Scale) {
    cv::Mat img = cv::imread("test_data/image.png", CV_LOAD_IMAGE_GRAYSCALE);
    ImgSize sz({4, 4, 8, 8, 4, 4});
    cv::Mat newImg = cropAndResize(img, sz);
    cv::Size expected(4, 4);
    EXPECT_EQ(newImg.size(), expected);
    EXPECT_EQ(newImg.at<uint8_t>(0, 0), 255);
    EXPECT_EQ(newImg.at<uint8_t>(2, 2), 0);
}
