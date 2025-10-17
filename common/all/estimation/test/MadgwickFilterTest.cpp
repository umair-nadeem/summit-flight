#include "estimation/MadgwickFilter.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MadgwickFilterTest : public ::testing::Test
{
protected:
   static constexpr float beta                 = 0.3f;
   static constexpr float gyro_bias_gain       = 0.5f;
   static constexpr float accel_tolerance_mps2 = 3.0f;
   static constexpr float dt_s                 = 0.004f;

   math::Vector3              accel{};
   math::Vector3              gyro{};
   estimation::MadgwickFilter ahrs_filter{beta, gyro_bias_gain, accel_tolerance_mps2};
};

using TestVector = std::tuple<math::Vector3, math::Vector3, math::Quaternion, uint32_t>;

class MadgwickFilterNedParametrizedTest : public MadgwickFilterTest, public ::testing::WithParamInterface<TestVector>
{
};

std::vector<TestVector> all_ned_inputs{
    {math::Vector3{0.0f, 0.0f, -9.8f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},
    {math::Vector3{-1.0f, 1.0f, 9.8f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999993f, 0.002849f, 0.002849f, 0.002000f}, 1u},
    {math::Vector3{-5.0f, 5.0f, 5.0f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999993f, 0.002849f, 0.002849f, 0.002000f}, 1u},
    {math::Vector3{8.0f, 8.0f, -8.0f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},
    {math::Vector3{10.0f, 10.0f, -10.0f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},

    {math::Vector3{0.0f, 0.0f, -9.8f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},
    {math::Vector3{-1.0f, 1.0f, 9.8f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999975f, 0.004848f, 0.004848f, 0.004000f}, 1u},
    {math::Vector3{-5.0f, 5.0f, 5.0f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999975f, 0.004848f, 0.004848f, 0.004000f}, 1u},
    {math::Vector3{8.0f, 8.0f, -8.0f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},
    {math::Vector3{10.0f, 10.0f, -10.0f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},

    {math::Vector3{0.0f, 0.0f, -9.8f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},
    {math::Vector3{-1.0f, 1.0f, 9.8f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999916f, -0.007151f, -0.007151f, -0.007999f}, 1u},
    {math::Vector3{-5.0f, 5.0f, 5.0f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999916f, -0.007151f, -0.007151f, -0.007999f}, 1u},
    {math::Vector3{8.0f, 8.0f, -8.0f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},
    {math::Vector3{10.0f, 10.0f, -10.0f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},

    {math::Vector3{0.0f, 0.0f, -9.8f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},
    {math::Vector3{-1.0f, 1.0f, 9.8f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.016842f, -0.0151456f, 0.015993f}, 1u},
    {math::Vector3{-5.0f, 5.0f, 5.0f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.016842f, -0.0151456f, 0.015993f}, 1u},
    {math::Vector3{8.0f, 8.0f, -8.0f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},
    {math::Vector3{10.0f, 10.0f, -10.0f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},

    {math::Vector3{0.0f, 0.0f, -9.8f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.927944f, -0.2835773f, 0.14272797f, -0.195276f}, 10u},
    {math::Vector3{-1.0f, 1.0f, 9.8f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.922345f, -0.296921f, 0.1550729f, -0.192530f}, 10u},
    {math::Vector3{-5.0f, 5.0f, 5.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.925702f, -0.286634f, 0.155634f, -0.1915549f}, 10u},
    {math::Vector3{8.0f, 8.0f, -8.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},
    {math::Vector3{10.0f, 10.0f, 10.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},

    {math::Vector3{-10.0f, -10.0f, 9.8f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},
    {math::Vector3{-8.0f, 0.0f, 9.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.922806f, -0.294988f, 0.1569957f, -0.191739f}, 10u},
    {math::Vector3{0.0f, -10.0f, -1.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.9212574f, -0.3029026f, 0.1456168f, -0.195781f}, 10u},
    {math::Vector3{-4.0f, -4.0f, -4.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.9219627f, -0.298227f, 0.154704f, -0.192643f}, 10u},
    {math::Vector3{0.0f, 0.0f, 0.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{-0.371640f, -0.713199f, 0.356599f, -0.475466f}, 50},

};

INSTANTIATE_TEST_SUITE_P(MadgwickFilterNedParametrizedTestInst,
                         MadgwickFilterNedParametrizedTest,
                         ::testing::ValuesIn(all_ned_inputs));

TEST_P(MadgwickFilterNedParametrizedTest, test_all_ned_inputs)
{
   const auto accel_data  = std::get<0>(GetParam());
   const auto gyro_data   = std::get<1>(GetParam());
   const auto ref_quat    = std::get<2>(GetParam());
   const auto num_updates = std::get<3>(GetParam());

   for (std::size_t i = 0; i < num_updates; i++)
   {
      ahrs_filter.update_in_ned_frame(accel_data, gyro_data, dt_s);
   }

   const auto test_quat = ahrs_filter.get_quaternion();

   EXPECT_NEAR(test_quat.w, ref_quat.w, 0.00001f);
   EXPECT_NEAR(test_quat.x, ref_quat.x, 0.00001f);
   EXPECT_NEAR(test_quat.y, ref_quat.y, 0.00001f);
   EXPECT_NEAR(test_quat.z, ref_quat.z, 0.00001f);
}

class MadgwickFilterEnuParametrizedTest : public MadgwickFilterTest, public ::testing::WithParamInterface<TestVector>
{
};

std::vector<TestVector> all_enu_inputs{
    {math::Vector3{0.0f, 0.0f, 9.8f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},
    {math::Vector3{1.0f, 1.0f, 9.8f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999993f, 0.002849f, 0.001151f, 0.002000f}, 1u},
    {math::Vector3{5.0f, 5.0f, 5.0f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999993f, 0.002849f, 0.001151f, 0.002000f}, 1u},
    {math::Vector3{8.0f, 8.0f, 8.0f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},
    {math::Vector3{10.0f, 10.0f, 10.0f}, math::Vector3{1.0f, 1.0f, 1.0f}, math::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},

    {math::Vector3{0.0f, 0.0f, 9.8f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},
    {math::Vector3{1.0f, 1.0f, 9.8f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999975f, 0.004848f, 0.003151f, 0.004000f}, 1u},
    {math::Vector3{5.0f, 5.0f, 5.0f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999975f, 0.004848f, 0.003151f, 0.004000f}, 1u},
    {math::Vector3{8.0f, 8.0f, 8.0f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},
    {math::Vector3{10.0f, 10.0f, 10.0f}, math::Vector3{2.0f, 2.0f, 2.0f}, math::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},

    {math::Vector3{0.0f, 0.0f, 9.8f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},
    {math::Vector3{1.0f, 1.0f, 9.8f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999903f, -0.007151f, -0.008848f, -0.007999f}, 1u},
    {math::Vector3{5.0f, 5.0f, 5.0f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999903f, -0.007151f, -0.008848f, -0.007999f}, 1u},
    {math::Vector3{8.0f, 8.0f, 8.0f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},
    {math::Vector3{10.0f, 10.0f, 10.0f}, math::Vector3{-4.0f, -4.0f, -4.0f}, math::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},

    {math::Vector3{0.0f, 0.0f, 9.8f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},
    {math::Vector3{1.0f, 1.0f, 9.8f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999588f, 0.016842f, -0.016842f, 0.015993f}, 1u},
    {math::Vector3{5.0f, 5.0f, 5.0f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999588f, 0.016842f, -0.016842f, 0.015993f}, 1u},
    {math::Vector3{8.0f, 8.0f, 8.0f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},
    {math::Vector3{10.0f, 10.0f, 10.0f}, math::Vector3{8.0f, -8.0f, 8.0f}, math::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},

    {math::Vector3{0.0f, 0.0f, 9.8f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.927944f, -0.2835773f, 0.14272797f, -0.195276f}, 10u},
    {math::Vector3{1.0f, 1.0f, 9.8f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.928214f, -0.283248f, 0.1408599f, -0.195823f}, 10u},
    {math::Vector3{5.0f, 5.0f, 5.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.9279481f, -0.284512f, 0.139243f, -0.196408f}, 10u},
    {math::Vector3{8.0f, 8.0f, 8.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},
    {math::Vector3{10.0f, 10.0f, 10.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},

    {math::Vector3{-10.0f, -10.0f, 9.8f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},
    {math::Vector3{-8.0f, 0.0f, 9.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.9249970f, -0.2886941f, 0.1560457f, -0.191540f}, 10u},
    {math::Vector3{0.0f, -10.0f, -1.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.9212301f, -0.3029152f, 0.1458612f, -0.195704f}, 10u},
    {math::Vector3{-4.0f, -4.0f, -4.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{0.92139089f, -0.3003195f, 0.1533119f, -0.193241f}, 10u},
    {math::Vector3{0.0f, 0.0f, 0.0f}, math::Vector3{-15.0f, 7.5f, -10.0f}, math::Quaternion{-0.371640f, -0.713199f, 0.356599f, -0.475466f}, 50},

};

INSTANTIATE_TEST_SUITE_P(MadgwickFilterEnuParametrizedTestInst,
                         MadgwickFilterEnuParametrizedTest,
                         ::testing::ValuesIn(all_enu_inputs));

TEST_P(MadgwickFilterEnuParametrizedTest, test_all_enu_inputs)
{
   const auto accel_data  = std::get<0>(GetParam());
   const auto gyro_data   = std::get<1>(GetParam());
   const auto ref_quat    = std::get<2>(GetParam());
   const auto num_updates = std::get<3>(GetParam());

   for (std::size_t i = 0; i < num_updates; i++)
   {
      ahrs_filter.update_in_enu_frame(accel_data, gyro_data, dt_s);
   }

   const auto test_quat = ahrs_filter.get_quaternion();

   EXPECT_NEAR(test_quat.w, ref_quat.w, 0.00001f);
   EXPECT_NEAR(test_quat.x, ref_quat.x, 0.00001f);
   EXPECT_NEAR(test_quat.y, ref_quat.y, 0.00001f);
   EXPECT_NEAR(test_quat.z, ref_quat.z, 0.00001f);
}
