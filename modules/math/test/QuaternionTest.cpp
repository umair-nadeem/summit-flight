#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "math/utility.hpp"

class QuaternionTest : public testing::Test
{
protected:
   static constexpr float tolerance = 1e-3f;
};

TEST_F(QuaternionTest, check_in_place_addition)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   a += b;

   EXPECT_NEAR(a[0], 0.0f, tolerance);
   EXPECT_NEAR(a[1], -49.0f, tolerance);
   EXPECT_NEAR(a[2], 114.0f, tolerance);
   EXPECT_NEAR(a[3], 11.1f, tolerance);
}

TEST_F(QuaternionTest, check_addition)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   const auto c = a + b;

   EXPECT_NEAR(c[0], 0.0f, tolerance);
   EXPECT_NEAR(c[1], -49.0f, tolerance);
   EXPECT_NEAR(c[2], 114.0f, tolerance);
   EXPECT_NEAR(c[3], 11.1f, tolerance);
}

TEST_F(QuaternionTest, check_in_place_subtraction)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   a -= b;

   EXPECT_NEAR(a[0], -10.0f, tolerance);
   EXPECT_NEAR(a[1], -151.2f, tolerance);
   EXPECT_NEAR(a[2], 86.0f, tolerance);
   EXPECT_NEAR(a[3], 9.1f, tolerance);
}

TEST_F(QuaternionTest, check_subtraction_result)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   const auto c = a - b;

   EXPECT_NEAR(c[0], -10.0f, tolerance);
   EXPECT_NEAR(c[1], -151.2f, tolerance);
   EXPECT_NEAR(c[2], 86.0f, tolerance);
   EXPECT_NEAR(c[3], 9.1f, tolerance);
}

TEST_F(QuaternionTest, check_multiplication_result)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   const auto c = a * 3.0f;

   EXPECT_NEAR(c[0], -15.0f, tolerance);
   EXPECT_NEAR(c[1], -300.3f, tolerance);
   EXPECT_NEAR(c[2], 300.0f, tolerance);
   EXPECT_NEAR(c[3], 30.3f, tolerance);
}

TEST_F(QuaternionTest, check_normalization)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   a.normalize();

   EXPECT_NEAR(a[0], -0.0352f, tolerance);
   EXPECT_NEAR(a[1], -0.7051f, tolerance);
   EXPECT_NEAR(a[2], 0.7040f, tolerance);
   EXPECT_NEAR(a[3], 0.0711f, tolerance);
}

TEST_F(QuaternionTest, check_euler_conversion)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   a.normalize();
   const auto euler = math::quaternion_to_euler(a);

   EXPECT_NEAR(euler.roll(), 2.9918f, tolerance);
   EXPECT_NEAR(euler.pitch(), 0.05077f, tolerance);
   EXPECT_NEAR(euler.yaw(), -1.5736f, tolerance);
}
