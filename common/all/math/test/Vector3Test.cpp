#include "math/Vector3.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class Vector3Test : public testing::Test
{
protected:
   static constexpr float tolerance = 1e-3f;
};

TEST_F(Vector3Test, check_in_place_addition)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   math::Vector3 b{51.1f, 14.0f, 1.0f};

   a += b;

   EXPECT_NEAR(a.x, -49.0f, tolerance);
   EXPECT_NEAR(a.y, 114.0f, tolerance);
   EXPECT_NEAR(a.z, 11.1f, tolerance);
}

TEST_F(Vector3Test, check_addition)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   math::Vector3 b{51.1f, 14.0f, 1.0f};

   const auto c = a + b;

   EXPECT_NEAR(c.x, -49.0f, tolerance);
   EXPECT_NEAR(c.y, 114.0f, tolerance);
   EXPECT_NEAR(c.z, 11.1f, tolerance);
}

TEST_F(Vector3Test, check_in_place_subtraction)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   math::Vector3 b{51.1f, 14.0f, 1.0f};

   a -= b;

   EXPECT_NEAR(a.x, -151.2f, tolerance);
   EXPECT_NEAR(a.y, 86.0f, tolerance);
   EXPECT_NEAR(a.z, 9.1f, tolerance);
}

TEST_F(Vector3Test, check_subtraction)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   math::Vector3 b{51.1f, 14.0f, 1.0f};

   const auto c = a - b;

   EXPECT_NEAR(c.x, -151.2f, tolerance);
   EXPECT_NEAR(c.y, 86.0f, tolerance);
   EXPECT_NEAR(c.z, 9.1f, tolerance);
}

TEST_F(Vector3Test, check_multiplication)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   math::Vector3 b{51.1f, 14.0f, 1.0f};

   const auto c = a * b;

   EXPECT_NEAR(c.x, -5115.11f, tolerance);
   EXPECT_NEAR(c.y, 1400.0f, tolerance);
   EXPECT_NEAR(c.z, 10.1f, tolerance);
}

TEST_F(Vector3Test, check_multiplication_with_scalar)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   const auto c = a * 3.0f;

   EXPECT_NEAR(c.x, -300.3f, tolerance);
   EXPECT_NEAR(c.y, 300.0f, tolerance);
   EXPECT_NEAR(c.z, 30.3f, tolerance);
}

TEST_F(Vector3Test, check_division_by_scalar)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   const auto c = a / 2.0f;

   EXPECT_NEAR(c.x, -50.05f, tolerance);
   EXPECT_NEAR(c.y, 50.0f, tolerance);
   EXPECT_NEAR(c.z, 5.05f, tolerance);
}

TEST_F(Vector3Test, check_normalization)
{
   math::Vector3 a{-100.1f, 100.0f, 10.1f};

   a.normalize();

   EXPECT_NEAR(a.x, -0.7056f, tolerance);
   EXPECT_NEAR(a.y, 0.7050f, tolerance);
   EXPECT_NEAR(a.z, 0.0712f, tolerance);
}
