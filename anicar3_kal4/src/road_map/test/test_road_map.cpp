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
#include "RoadMap.hpp"
#include "gtest/gtest.h"

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(TestGroupName, TestName) {
// TODO: Add your test code here
//}

TEST(RoadMap, mapPropertyFunctions)
{
    RoadMap roadMap(49, 8.42);
    // add a trajectory along the x axis
    roadMap.addTrajectory({-3, -2, -1, 0, 1, 2, 3}, {0, 0, 0, 0, 0, 0, 0});
    roadMap.addTrajectory({-100,100},{100,100}); //far away trajectory that should not affect result

    // get some closest points
    double heading, curv, d;
    EXPECT_TRUE(roadMap.getControlReferenceMeters(0, 1, d, heading, curv, -1));
    EXPECT_EQ(-1, d);
    EXPECT_EQ(curv, 0);
    EXPECT_EQ(heading, 0);

    int32_t t_id{-1}, v_id{0};
    EXPECT_TRUE(roadMap.findClosestVertexMeters(0, 1, t_id, v_id));
    EXPECT_EQ(t_id, 0);
    EXPECT_EQ(v_id, 3);

    double s;
    t_id = -1; //search all trajectories
    EXPECT_TRUE(roadMap.getGeodesicTravelDistanceMeters(0.5, 1, t_id, v_id, s, d));
    EXPECT_EQ(t_id, 0);
    EXPECT_EQ(v_id, 3);
    EXPECT_DOUBLE_EQ(d, 3.5);
    EXPECT_DOUBLE_EQ(s, 0.5);
}

TEST(RoadMap, CopyAndMove)
{
    RoadMap copyTo(49, 8.42);

    {
        // create object and copy it outside our scope
        RoadMap copyFrom(49, 8.42);
        copyFrom.addTrajectory({-3, -2, -1, 0, 1, 2, 3}, {0, 0, 0, 0, 0, 0, 0});
        int32_t t_id{-1}, v_id;
        copyTo = copyFrom;

        //test if object is valid after copy
        copyFrom.findClosestVertexMeters(0, 1, t_id, v_id);
        EXPECT_EQ(t_id, 0);
        EXPECT_EQ(v_id, 3);
    }
    //test if object is valid after deleting other object
    int32_t t_id{-1}, v_id;
    copyTo.findClosestVertexMeters(0, 1, t_id, v_id);
    EXPECT_EQ(t_id, 0);
    EXPECT_EQ(v_id, 3);

    // create temporary and move it outside of scope
    {
        RoadMap moveFrom(49, 8.42);
        moveFrom.addTrajectory({-3, -2, -1, 0, 1, 2, 3}, {0, 0, 0, 0, 0, 0, 0});
        copyTo = std::move(moveFrom);
    }

    t_id = -1;
    copyTo.findClosestVertexMeters(0, 1, t_id, v_id);
    EXPECT_EQ(t_id, 0);
    EXPECT_EQ(v_id, 3);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
