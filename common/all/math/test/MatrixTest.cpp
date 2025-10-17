#include "math/Matrix.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MatrixTest : public testing::Test
{
protected:
   static constexpr float tolerance = 1e-3f;
};

TEST_F(MatrixTest, check_reset)
{
   math::Matrix<float, 4u, 4u> matrix{};
   matrix.reset();

   EXPECT_NEAR(matrix.at(0, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(1, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(2, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(3, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 3), 0.0f, tolerance);
}

TEST_F(MatrixTest, check_identity_1)
{
   math::Matrix<float, 4u, 4u> matrix{};
   matrix.set_identity();

   EXPECT_NEAR(matrix.at(0, 0), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(1, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 1), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(2, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 2), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(3, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 3), 1.0f, tolerance);
}

TEST_F(MatrixTest, check_identity_2)
{
   const auto matrix = math::Matrix<float, 4, 4>::Identity();

   EXPECT_NEAR(matrix.at(0, 0), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(1, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 1), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(2, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 2), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(3, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 3), 1.0f, tolerance);
}

TEST_F(MatrixTest, check_arithmetic)
{
   math::Matrix<float, 4u, 4u> matrix{};
   matrix.set_identity();
   matrix += 3.0f;
   matrix -= 5.0f;
   matrix *= 12.0f;
   matrix /= 4.0f;

   EXPECT_NEAR(matrix.at(0, 0), -3.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 1), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 2), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 3), -6.0f, tolerance);

   EXPECT_NEAR(matrix.at(1, 0), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 1), -3.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 2), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 3), -6.0f, tolerance);

   EXPECT_NEAR(matrix.at(2, 0), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 1), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 2), -3.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 3), -6.0f, tolerance);

   EXPECT_NEAR(matrix.at(3, 0), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 1), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 2), -6.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 3), -3.0f, tolerance);
}

TEST_F(MatrixTest, check_transpose)
{
   // matrix
   // [-1,   0, -52, 345.5]
   // [ 0,  67,  51,     0]
   // [ 7,   0,   0,  11.5]
   // [-9.9  0,   0,   9.9]

   math::Matrix<float, 4u, 4u> matrix{};
   matrix.at(0, 0) = -1.0f;
   matrix.at(0, 2) = -52.0f;
   matrix.at(0, 3) = 345.5f;
   matrix.at(1, 1) = 67.0f;
   matrix.at(1, 2) = 51.0f;
   matrix.at(2, 0) = 7.0f;
   matrix.at(2, 3) = 11.5f;
   matrix.at(3, 0) = -9.9f;
   matrix.at(3, 3) = 9.9f;

   math::Matrix<float, 4u, 4u> other = matrix.transpose();

   EXPECT_NEAR(other.at(0, 0), -1.0f, tolerance);
   EXPECT_NEAR(other.at(0, 1), 0.0f, tolerance);
   EXPECT_NEAR(other.at(0, 2), 7.0f, tolerance);
   EXPECT_NEAR(other.at(0, 3), -9.9f, tolerance);

   EXPECT_NEAR(other.at(1, 0), 0.0f, tolerance);
   EXPECT_NEAR(other.at(1, 1), 67.0f, tolerance);
   EXPECT_NEAR(other.at(1, 2), 0.0f, tolerance);
   EXPECT_NEAR(other.at(1, 3), 0.0f, tolerance);

   EXPECT_NEAR(other.at(2, 0), -52.0f, tolerance);
   EXPECT_NEAR(other.at(2, 1), 51.0f, tolerance);
   EXPECT_NEAR(other.at(2, 2), 0.0f, tolerance);
   EXPECT_NEAR(other.at(2, 3), 0.0f, tolerance);

   EXPECT_NEAR(other.at(3, 0), 345.5f, tolerance);
   EXPECT_NEAR(other.at(3, 1), 0.0f, tolerance);
   EXPECT_NEAR(other.at(3, 2), 11.5f, tolerance);
   EXPECT_NEAR(other.at(3, 3), 9.9f, tolerance);
}

TEST_F(MatrixTest, check_matrix_addition)
{
   auto matrix = math::Matrix<float, 4u, 4u>::Identity();

   // other
   // [-1,   0, -52, 345.5]
   // [ 0,  67,  51,     0]
   // [ 7,   0,   0,  11.5]
   // [-9.9  0,   0,   9.9]

   math::Matrix<float, 4u, 4u> other{};
   other.at(0, 0) = -1.0f;
   other.at(0, 2) = -52.0f;
   other.at(0, 3) = 345.5f;
   other.at(1, 1) = 67.0f;
   other.at(1, 2) = 51.0f;
   other.at(2, 0) = 7.0f;
   other.at(2, 3) = 11.5f;
   other.at(3, 0) = -9.9f;
   other.at(3, 3) = 9.9f;

   matrix += other;

   EXPECT_NEAR(matrix.at(0, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 2), -52.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 3), 345.5f, tolerance);

   EXPECT_NEAR(matrix.at(1, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 1), 68.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 2), 51.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(2, 0), 7.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 2), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 3), 11.5f, tolerance);

   EXPECT_NEAR(matrix.at(3, 0), -9.9f, tolerance);
   EXPECT_NEAR(matrix.at(3, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 3), 10.9f, tolerance);
}

TEST_F(MatrixTest, check_matrix_subtraction)
{
   auto matrix = math::Matrix<float, 4u, 4u>::Identity();

   // other
   // [-1,   0, -52, 345.5]
   // [ 0,  67,  51,     0]
   // [ 7,   0,   0,  11.5]
   // [-9.9  0,   0,   9.9]

   math::Matrix<float, 4u, 4u> other{};
   other.at(0, 0) = -1.0f;
   other.at(0, 2) = -52.0f;
   other.at(0, 3) = 345.5f;
   other.at(1, 1) = 67.0f;
   other.at(1, 2) = 51.0f;
   other.at(2, 0) = 7.0f;
   other.at(2, 3) = 11.5f;
   other.at(3, 0) = -9.9f;
   other.at(3, 3) = 9.9f;

   matrix -= other;

   EXPECT_NEAR(matrix.at(0, 0), 2.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 2), 52.0f, tolerance);
   EXPECT_NEAR(matrix.at(0, 3), -345.5f, tolerance);

   EXPECT_NEAR(matrix.at(1, 0), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 1), -66.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 2), -51.0f, tolerance);
   EXPECT_NEAR(matrix.at(1, 3), 0.0f, tolerance);

   EXPECT_NEAR(matrix.at(2, 0), -7.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 2), 1.0f, tolerance);
   EXPECT_NEAR(matrix.at(2, 3), -11.5f, tolerance);

   EXPECT_NEAR(matrix.at(3, 0), 9.9f, tolerance);
   EXPECT_NEAR(matrix.at(3, 1), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 2), 0.0f, tolerance);
   EXPECT_NEAR(matrix.at(3, 3), -8.9f, tolerance);
}

TEST_F(MatrixTest, check_matrix_4_4_multiplication)
{
   // matrix=
   // [-1,   0, -52, 345.5]
   // [ 0,  67,  51,     0]
   // [ 7,   0,   0,  11.5]
   // [-9.9  0,   0,   9.9]

   math::Matrix<float, 4u, 4u> matrix{};
   matrix.at(0, 0) = -1.0f;
   matrix.at(0, 2) = -52.0f;
   matrix.at(0, 3) = 345.5f;
   matrix.at(1, 1) = 67.0f;
   matrix.at(1, 2) = 51.0f;
   matrix.at(2, 0) = 7.0f;
   matrix.at(2, 3) = 11.5f;
   matrix.at(3, 0) = -9.9f;
   matrix.at(3, 3) = 9.9f;

   // other=
   // [11,  100,  27,  99.9,   5,  -1]
   // [ 1,    1   -1,    20,  77,   8]
   // [ 3,    5,   7,   -19,  -2,   4]
   // [-9.9  10,  10,   9.9,  41,   0]

   math::Matrix<float, 4u, 6u> other{};
   other.at(0, 0) = 11.0f;
   other.at(0, 1) = 100.0f;
   other.at(0, 2) = 27.0f;
   other.at(0, 3) = 99.9f;
   other.at(0, 4) = 5.0f;
   other.at(0, 5) = -1.0f;

   other.at(1, 0) = 1.0f;
   other.at(1, 1) = 1.0f;
   other.at(1, 2) = -1.0f;
   other.at(1, 3) = 20.0f;
   other.at(1, 4) = 77.0f;
   other.at(1, 5) = 8.0f;

   other.at(2, 0) = 3.0f;
   other.at(2, 1) = 5.0f;
   other.at(2, 2) = 7.0f;
   other.at(2, 3) = -19.0f;
   other.at(2, 4) = -2.0f;
   other.at(2, 5) = 4.0f;

   other.at(3, 0) = -9.9f;
   other.at(3, 1) = 10.0f;
   other.at(3, 2) = 10.0f;
   other.at(3, 3) = 9.9f;
   other.at(3, 4) = 41.0f;
   other.at(3, 5) = 0.0f;

   math::Matrix<float, 4u, 6u> result = matrix * other;

   EXPECT_NEAR(result.at(0, 0), -3587.45f, tolerance);
   EXPECT_NEAR(result.at(0, 1), 3095.0f, tolerance);
   EXPECT_NEAR(result.at(0, 2), 3064.0f, tolerance);
   EXPECT_NEAR(result.at(0, 3), 4308.55f, tolerance);
   EXPECT_NEAR(result.at(0, 4), 14264.5f, tolerance);
   EXPECT_NEAR(result.at(0, 5), -207.0f, tolerance);

   EXPECT_NEAR(result.at(1, 0), 220.0f, tolerance);
   EXPECT_NEAR(result.at(1, 1), 322.0f, tolerance);
   EXPECT_NEAR(result.at(1, 2), 290.0f, tolerance);
   EXPECT_NEAR(result.at(1, 3), 371.0f, tolerance);
   EXPECT_NEAR(result.at(1, 4), 5057.0f, tolerance);
   EXPECT_NEAR(result.at(1, 5), 740.0f, tolerance);

   EXPECT_NEAR(result.at(2, 0), -36.85f, tolerance);
   EXPECT_NEAR(result.at(2, 1), 815.0f, tolerance);
   EXPECT_NEAR(result.at(2, 2), 304.0f, tolerance);
   EXPECT_NEAR(result.at(2, 3), 813.15f, tolerance);
   EXPECT_NEAR(result.at(2, 4), 506.5f, tolerance);
   EXPECT_NEAR(result.at(2, 5), -7.0f, tolerance);

   EXPECT_NEAR(result.at(3, 0), -206.91f, tolerance);
   EXPECT_NEAR(result.at(3, 1), -891.0f, tolerance);
   EXPECT_NEAR(result.at(3, 2), -168.3f, tolerance);
   EXPECT_NEAR(result.at(3, 3), -891.0f, tolerance);
   EXPECT_NEAR(result.at(3, 4), 356.4f, tolerance);
   EXPECT_NEAR(result.at(3, 5), 9.9f, tolerance);
}

TEST_F(MatrixTest, check_matrix_3_2_multiplication)
{
   // matrix=
   // [-1, -17]
   // [ 0,  67]
   // [ 7,   9]

   math::Matrix<int, 3u, 2u> matrix{};
   matrix.at(0, 0) = -1;
   matrix.at(0, 1) = -17;
   matrix.at(1, 0) = 0;
   matrix.at(1, 1) = 67;
   matrix.at(2, 0) = 7;
   matrix.at(2, 1) = 9;

   // other=
   // [5,  11,  -9]
   // [1, 107,  -2]

   math::Matrix<int, 2u, 3u> other{};
   other.at(0, 0) = 5;
   other.at(0, 1) = 11;
   other.at(0, 2) = -9;
   other.at(1, 0) = 1;
   other.at(1, 1) = 107;
   other.at(1, 2) = -2;

   const math::Matrix<int, 3u, 3u> result = matrix * other;

   EXPECT_EQ(result.at(0, 0), -22);
   EXPECT_EQ(result.at(0, 1), -1830);
   EXPECT_EQ(result.at(0, 2), 43);
   EXPECT_EQ(result.at(1, 0), 67);
   EXPECT_EQ(result.at(1, 1), 7169);
   EXPECT_EQ(result.at(1, 2), -134);
   EXPECT_EQ(result.at(2, 0), 44);
   EXPECT_EQ(result.at(2, 1), 1040);
   EXPECT_EQ(result.at(2, 2), -81);
}

TEST_F(MatrixTest, check_matrix_2_3_multiplication)
{
   // matrix=
   // [5,  11,  -9]
   // [1, 107,  -2]

   math::Matrix<int, 2u, 3u> matrix{};
   matrix.at(0, 0) = 5;
   matrix.at(0, 1) = 11;
   matrix.at(0, 2) = -9;
   matrix.at(1, 0) = 1;
   matrix.at(1, 1) = 107;
   matrix.at(1, 2) = -2;

   // other=
   // [-1, -17]
   // [ 0,  67]
   // [ 7,   9]

   math::Matrix<int, 3u, 2u> other{};
   other.at(0, 0) = -1;
   other.at(0, 1) = -17;
   other.at(1, 0) = 0;
   other.at(1, 1) = 67;
   other.at(2, 0) = 7;
   other.at(2, 1) = 9;

   const math::Matrix<int, 2u, 2u> result = matrix * other;

   EXPECT_EQ(result.at(0, 0), -68);
   EXPECT_EQ(result.at(0, 1), 571);
   EXPECT_EQ(result.at(1, 0), -15);
   EXPECT_EQ(result.at(1, 1), 7134);
}
