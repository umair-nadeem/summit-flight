#include "boundaries/SharedData.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class SharedDataTest : public testing::Test
{

protected:
   struct TestData
   {
      std::optional<float> optional{};
      int                  concrete{};   // cppcheck-suppress unusedStructMember
   };

   using SharedDataStorage = boundaries::SharedData<TestData>;

   SharedDataStorage shared_data_storage{};
};

TEST_F(SharedDataTest, get_latest_without_updating)
{
   const SharedDataStorage::Sample latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 0);
   EXPECT_EQ(latest.data.concrete, 0);
   EXPECT_EQ(latest.data.optional.has_value(), false);
}

TEST_F(SharedDataTest, update_timestampt_without_updating_data)
{
   shared_data_storage.update_latest(TestData{}, 1u);

   const SharedDataStorage::Sample latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 1u);
   EXPECT_EQ(latest.data.concrete, 0);
   EXPECT_EQ(latest.data.optional.has_value(), false);
}

TEST_F(SharedDataTest, get_latest_after_updating_once)
{
   shared_data_storage.update_latest(TestData{1.0f, 1}, 1u);

   const SharedDataStorage::Sample latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 1u);
   EXPECT_EQ(latest.data.concrete, 1);
   EXPECT_NEAR(latest.data.optional.value(), 1.0f, 0.0000001f);
}

TEST_F(SharedDataTest, get_latest_after_updating_twice)
{
   shared_data_storage.update_latest(TestData{1.0f, 1}, 1u);
   shared_data_storage.update_latest(TestData{2.0f, 2}, 2u);

   SharedDataStorage::Sample latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 2u);
   EXPECT_EQ(latest.data.concrete, 2);
   EXPECT_NEAR(latest.data.optional.value(), 2.0f, 0.0000001f);

   // attempt again to get latest
   latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 2u);
   EXPECT_EQ(latest.data.concrete, 2);
   EXPECT_NEAR(latest.data.optional.value(), 2.0f, 0.0000001f);
}

TEST_F(SharedDataTest, get_latest_after_updating_three_times)
{
   shared_data_storage.update_latest(TestData{1.0f, 1}, 1u);
   shared_data_storage.update_latest(TestData{2.0f, 2}, 2u);
   shared_data_storage.update_latest(TestData{3.0f, 3}, 3u);

   SharedDataStorage::Sample latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 3u);
   EXPECT_EQ(latest.data.concrete, 3);
   EXPECT_NEAR(latest.data.optional.value(), 3.0f, 0.0000001f);

   // attempt again to get latest
   latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 3u);
   EXPECT_EQ(latest.data.concrete, 3);
   EXPECT_NEAR(latest.data.optional.value(), 3.0f, 0.0000001f);
}

TEST_F(SharedDataTest, get_latest_after_updating_four_times)
{
   shared_data_storage.update_latest(TestData{1.0f, 1}, 1u);
   shared_data_storage.update_latest(TestData{2.0f, 2}, 2u);
   shared_data_storage.update_latest(TestData{3.0f, 3}, 3u);
   shared_data_storage.update_latest(TestData{4.0f, 4}, 4u);

   SharedDataStorage::Sample latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 4u);
   EXPECT_EQ(latest.data.concrete, 4);
   EXPECT_NEAR(latest.data.optional.value(), 4.0f, 0.0000001f);

   // attempt again to get latest
   latest = shared_data_storage.get_latest();

   EXPECT_EQ(latest.timestamp_ms, 4u);
   EXPECT_EQ(latest.data.concrete, 4);
   EXPECT_NEAR(latest.data.optional.value(), 4.0f, 0.0000001f);
}
