#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include "uav_controller/controller_sep.hpp"

// Make sure that gtest is working properly
TEST(TestGtest, testFail)
{
    ASSERT_EQ(1.0, 0.0);
}

TEST(TestGtest, testPass)
{
    ASSERT_EQ(1.0, 1.0);
}

TEST(TestController, testAttitudeController)
{
    Vector3d M_known;
    M_known << 2.0, 2.0, 3.0;

    Vector3d M;
    Vector3d W = Vector3d::Zero();
    Vector3d Wd = Vector3d::Zero();
    Vector3d Wddot = Vector3d::Zero();
    Matrix3d R = Matrix3d::Zero();
    Matrix3d Rd = Matrix3d::Zero();

    controller_sep::AttitudeControl(W,Wd,Wddot, R, Rd, M);

    ASSERT_TRUE(M_known.isApprox(M));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "controller_tester");
    return RUN_ALL_TESTS();
}
