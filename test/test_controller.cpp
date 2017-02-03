#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include "odroid/controller_sep.hpp"

// Make sure that gtest is working properly
TEST(TestGtest, testFail)
{
    ASSERT_EQ(1.0, 0.0);
}

TEST(TestGtest, testPass)
{
    ASSERT_EQ(1.0, 1.0);
}

TEST(TestController, testSpherical)
{
    controller_sep ctl;
    int b = ctl.add(1,2);
    std::cout << b << std::endl;

    ASSERT_EQ(3, 3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "controller_tester");
    return RUN_ALL_TESTS();
}