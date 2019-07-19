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
#include "local_geographic_cs.hpp"
#include "gtest/gtest.h"

TEST(TestRegularUsageNoOffset, TestForwardConversion) {
    double lat_init = 49.01439;
    double lon_init = 8.41722;
    gnss::LocalGeographicCS cs(lat_init, lon_init, false);
    double x, y;
    cs.ll2xy(lat_init, lon_init, x, y);
    ASSERT_NEAR(x, 457386.3823855548, 0.0001);
    ASSERT_NEAR(y, 5429219.050748879, 0.0001);
}

TEST(TestRegularUsageNoOffset, TestForwardAndReverseConversion) {
    double lat_init = 49.01439;
    double lon_init = 8.41722;
    gnss::LocalGeographicCS cs(lat_init, lon_init, false);
    double x, y;
    cs.ll2xy(lat_init, lon_init, x, y);
    ASSERT_NEAR(x, 457386.3823855548, 0.0001);
    ASSERT_NEAR(y, 5429219.050748879, 0.0001);

    double lat, lon;
    cs.xy2ll(x, y, lat, lon);
    ASSERT_DOUBLE_EQ(lat_init, lat);
    ASSERT_DOUBLE_EQ(lon_init, lon);
}

TEST(TestRegularUsageWithOffset, TestForwardConversion) {
    double lat_init = 49.01439;
    double lon_init = 8.41722;
    gnss::LocalGeographicCS cs(lat_init, lon_init);
    double x, y;
    cs.ll2xy(lat_init, lon_init, x, y);
    ASSERT_NEAR(x, 0, 0.0001);
    ASSERT_NEAR(y, 0, 0.0001);
}

TEST(TestRegularUsageWithOffset, TestForwardAndReverseConversion) {
    double lat_init = 49.01439;
    double lon_init = 8.41722;
    gnss::LocalGeographicCS cs(lat_init, lon_init);
    double x, y;
    cs.ll2xy(lat_init, lon_init, x, y);
    ASSERT_NEAR(x, 0, 0.0001);
    ASSERT_NEAR(y, 0, 0.0001);

    double lat, lon;
    cs.xy2ll(x, y, lat, lon);
    ASSERT_DOUBLE_EQ(lat_init, lat);
    ASSERT_DOUBLE_EQ(lon_init, lon);
}

TEST(TestEdgeCases, TestOutOfHemisphere) {
    double lat_init = 49.01439; // northern hemisphere
    double lon_init = 8.41722;
    gnss::LocalGeographicCS cs(lat_init, lon_init, false);
    double x, y;
    double lat_test = -lat_init; // southern hemisphere
    ASSERT_THROW(cs.ll2xy(lat_test, lon_init, x, y), std::runtime_error);
}

TEST(TestEdgeCases, TestOutOfZone) {
    double lat_init = 49.01439;
    double lon_init = 8.41722; // UTM zone 32
    gnss::LocalGeographicCS cs(lat_init, lon_init, false);
    double x, y;
    double lon_test = 5.; // UTM zone 31
    ASSERT_THROW(cs.ll2xy(lat_init, lon_test, x, y), std::runtime_error);
}

TEST(TestEdgeCases, TestCompletelyOutOfZoneReverse) {
    double lat_init = 49.01439;
    double lon_init = 8.41722; // UTM zone 32
    gnss::LocalGeographicCS cs(lat_init, lon_init, false);
    double x = 0.0;
    double y = 0.0;  // outside UTM zone 32
    ASSERT_THROW(cs.xy2ll(x, y, lat_init, lon_init);, std::runtime_error);
}

TEST(TestEdgeCases, TestSlightlyOutOfZoneReverse) {
    double lat_init = 49.01439;
    double lon_init = 8.41722; // UTM zone 32
    gnss::LocalGeographicCS cs(lat_init, lon_init, false);
    double x = 900000.0; // outside UTM zone 32
    double y = 5000000.0;
    ASSERT_THROW(cs.xy2ll(x, y, lat_init, lon_init);, std::runtime_error);
}
