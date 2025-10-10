#include "estimation/MadgwickFilter.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MadgwickFilterTest : public ::testing::Test
{
protected:
   static constexpr float beta                 = 0.3f;
   static constexpr float gyro_bias_gain       = 0.5f;
   static constexpr float accel_tolerance_mps2 = 3.0f;
   static constexpr float sampling_period_s    = 4u / 1000.0f;

   physics::Vector3           accel{};
   physics::Vector3           gyro{};
   estimation::MadgwickFilter ahrs_filter{beta, gyro_bias_gain, accel_tolerance_mps2, sampling_period_s};
};

using TestVector = std::tuple<physics::Vector3, physics::Vector3, physics::Quaternion, uint32_t>;

class MadgwickFilterParametrizedTest : public MadgwickFilterTest, public ::testing::WithParamInterface<TestVector>
{
};

std::vector<TestVector> all_inputs{
    {physics::Vector3{0.0f, 0.0f, 9.8f}, physics::Vector3{1.0f, 1.0f, 1.0f}, physics::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},
    {physics::Vector3{1.0f, 1.0f, 9.8f}, physics::Vector3{1.0f, 1.0f, 1.0f}, physics::Quaternion{0.999993f, 0.002849f, 0.001151f, 0.002000f}, 1u},
    {physics::Vector3{5.0f, 5.0f, 5.0f}, physics::Vector3{1.0f, 1.0f, 1.0f}, physics::Quaternion{0.999993f, 0.002849f, 0.001151f, 0.002000f}, 1u},
    {physics::Vector3{8.0f, 8.0f, 8.0f}, physics::Vector3{1.0f, 1.0f, 1.0f}, physics::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},
    {physics::Vector3{10.0f, 10.0f, 10.0f}, physics::Vector3{1.0f, 1.0f, 1.0f}, physics::Quaternion{0.999994f, 0.002000f, 0.002000f, 0.002000f}, 1u},

    {physics::Vector3{0.0f, 0.0f, 9.8f}, physics::Vector3{2.0f, 2.0f, 2.0f}, physics::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},
    {physics::Vector3{1.0f, 1.0f, 9.8f}, physics::Vector3{2.0f, 2.0f, 2.0f}, physics::Quaternion{0.999975f, 0.004848f, 0.003151f, 0.004000f}, 1u},
    {physics::Vector3{5.0f, 5.0f, 5.0f}, physics::Vector3{2.0f, 2.0f, 2.0f}, physics::Quaternion{0.999975f, 0.004848f, 0.003151f, 0.004000f}, 1u},
    {physics::Vector3{8.0f, 8.0f, 8.0f}, physics::Vector3{2.0f, 2.0f, 2.0f}, physics::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},
    {physics::Vector3{10.0f, 10.0f, 10.0f}, physics::Vector3{2.0f, 2.0f, 2.0f}, physics::Quaternion{0.999976f, 0.004000f, 0.004000f, 0.004000f}, 1u},

    {physics::Vector3{0.0f, 0.0f, 9.8f}, physics::Vector3{-4.0f, -4.0f, -4.0f}, physics::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},
    {physics::Vector3{1.0f, 1.0f, 9.8f}, physics::Vector3{-4.0f, -4.0f, -4.0f}, physics::Quaternion{0.999903f, -0.007151f, -0.008848f, -0.007999f}, 1u},
    {physics::Vector3{5.0f, 5.0f, 5.0f}, physics::Vector3{-4.0f, -4.0f, -4.0f}, physics::Quaternion{0.999903f, -0.007151f, -0.008848f, -0.007999f}, 1u},
    {physics::Vector3{8.0f, 8.0f, 8.0f}, physics::Vector3{-4.0f, -4.0f, -4.0f}, physics::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},
    {physics::Vector3{10.0f, 10.0f, 10.0f}, physics::Vector3{-4.0f, -4.0f, -4.0f}, physics::Quaternion{0.999904f, -0.007999f, -0.007999f, -0.007999f}, 1u},

    {physics::Vector3{0.0f, 0.0f, 9.8f}, physics::Vector3{8.0f, -8.0f, 8.0f}, physics::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},
    {physics::Vector3{1.0f, 1.0f, 9.8f}, physics::Vector3{8.0f, -8.0f, 8.0f}, physics::Quaternion{0.999588f, 0.016842f, -0.016842f, 0.015993f}, 1u},
    {physics::Vector3{5.0f, 5.0f, 5.0f}, physics::Vector3{8.0f, -8.0f, 8.0f}, physics::Quaternion{0.999588f, 0.016842f, -0.016842f, 0.015993f}, 1u},
    {physics::Vector3{8.0f, 8.0f, 8.0f}, physics::Vector3{8.0f, -8.0f, 8.0f}, physics::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},
    {physics::Vector3{10.0f, 10.0f, 10.0f}, physics::Vector3{8.0f, -8.0f, 8.0f}, physics::Quaternion{0.999616f, 0.015994f, -0.015994f, 0.015994f}, 1u},

    {physics::Vector3{0.0f, 0.0f, 9.8f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.927944f, -0.2835773f, 0.14272797f, -0.195276f}, 10u},
    {physics::Vector3{1.0f, 1.0f, 9.8f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.928214f, -0.283248f, 0.1408599f, -0.195823f}, 10u},
    {physics::Vector3{5.0f, 5.0f, 5.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.9279481f, -0.284512f, 0.139243f, -0.196408f}, 10u},
    {physics::Vector3{8.0f, 8.0f, 8.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},
    {physics::Vector3{10.0f, 10.0f, 10.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},

    {physics::Vector3{-10.0f, -10.0f, 9.8f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.924790f, -0.292292f, 0.146146f, -0.194861f}, 10u},
    {physics::Vector3{-8.0f, 0.0f, 9.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.9249970f, -0.2886941f, 0.1560457f, -0.191540f}, 10u},
    {physics::Vector3{0.0f, -10.0f, -1.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.9212301f, -0.3029152f, 0.1458612f, -0.195704f}, 10u},
    {physics::Vector3{-4.0f, -4.0f, -4.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{0.92139089f, -0.3003195f, 0.1533119f, -0.193241f}, 10u},
    {physics::Vector3{0.0f, 0.0f, 0.0f}, physics::Vector3{-15.0f, 7.5f, -10.0f}, physics::Quaternion{-0.371640f, -0.713199f, 0.356599f, -0.475466f}, 50},

};

INSTANTIATE_TEST_SUITE_P(MadgwickFilterParametrizedTestInst,
                         MadgwickFilterParametrizedTest,
                         ::testing::ValuesIn(all_inputs));

TEST_P(MadgwickFilterParametrizedTest, test_all_inputs)
{
   const auto accel_data  = std::get<0>(GetParam());
   const auto gyro_data   = std::get<1>(GetParam());
   const auto ref_quat    = std::get<2>(GetParam());
   const auto num_updates = std::get<3>(GetParam());

   for (std::size_t i = 0; i < num_updates; i++)
   {
      ahrs_filter.update(accel_data, gyro_data);
   }

   const auto test_quat = ahrs_filter.get_quaternion();

   EXPECT_NEAR(test_quat.w, ref_quat.w, 0.00001f);
   EXPECT_NEAR(test_quat.x, ref_quat.x, 0.00001f);
   EXPECT_NEAR(test_quat.y, ref_quat.y, 0.00001f);
   EXPECT_NEAR(test_quat.z, ref_quat.z, 0.00001f);
}
