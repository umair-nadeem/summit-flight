#include "math/Quaternion.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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

   EXPECT_NEAR(a.w, 0.0f, tolerance);
   EXPECT_NEAR(a.x, -49.0f, tolerance);
   EXPECT_NEAR(a.y, 114.0f, tolerance);
   EXPECT_NEAR(a.z, 11.1f, tolerance);
}

TEST_F(QuaternionTest, check_addition)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   const auto c = a + b;

   EXPECT_NEAR(c.w, 0.0f, tolerance);
   EXPECT_NEAR(c.x, -49.0f, tolerance);
   EXPECT_NEAR(c.y, 114.0f, tolerance);
   EXPECT_NEAR(c.z, 11.1f, tolerance);
}

TEST_F(QuaternionTest, check_in_place_subtraction)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   a -= b;

   EXPECT_NEAR(a.w, -10.0f, tolerance);
   EXPECT_NEAR(a.x, -151.2f, tolerance);
   EXPECT_NEAR(a.y, 86.0f, tolerance);
   EXPECT_NEAR(a.z, 9.1f, tolerance);
}

TEST_F(QuaternionTest, check_subtraction_result)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   math::Quaternion b{5.0f, 51.1f, 14.0f, 1.0f};

   const auto c = a - b;

   EXPECT_NEAR(c.w, -10.0f, tolerance);
   EXPECT_NEAR(c.x, -151.2f, tolerance);
   EXPECT_NEAR(c.y, 86.0f, tolerance);
   EXPECT_NEAR(c.z, 9.1f, tolerance);
}

TEST_F(QuaternionTest, check_multiplication_result)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   const auto c = a * 3.0f;

   EXPECT_NEAR(c.w, -15.0f, tolerance);
   EXPECT_NEAR(c.x, -300.3f, tolerance);
   EXPECT_NEAR(c.y, 300.0f, tolerance);
   EXPECT_NEAR(c.z, 30.3f, tolerance);
}

TEST_F(QuaternionTest, check_normalization)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   a.normalize();

   EXPECT_NEAR(a.w, -0.0352f, tolerance);
   EXPECT_NEAR(a.x, -0.7051f, tolerance);
   EXPECT_NEAR(a.y, 0.7040f, tolerance);
   EXPECT_NEAR(a.z, 0.0711f, tolerance);
}

TEST_F(QuaternionTest, check_euler_conversion)
{
   math::Quaternion a{-5.0f, -100.1f, 100.0f, 10.1f};

   a.normalize();
   const auto euler = a.to_euler();

   EXPECT_NEAR(euler.x, 2.9918f, tolerance);
   EXPECT_NEAR(euler.y, 0.05077f, tolerance);
   EXPECT_NEAR(euler.z, -1.5736f, tolerance);
}
