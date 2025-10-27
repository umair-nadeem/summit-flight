#include "aeromight_imu/ImuDriverExecutor.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/Executable.hpp"
#include "mocks/common/Logger.hpp"
#include "mocks/pcb_component/Led.hpp"
#include "mocks/rtos/NotificationWaiter.hpp"

class ImuDriverExecutorTest : public testing::Test
{
protected:
   mocks::common::Executable                                                   mpu_driver_mock{};
   mocks::rtos::NotificationWaiter<aeromight_boundaries::ImuNotificationFlags> notification_waiter{};
   mocks::pcb_component::Led                                                   led_mock{};
   mocks::common::Logger                                                       logger{"test"};
   uint32_t                                                                    period_ms{1u};

   aeromight_imu::ImuDriverExecutor<decltype(mpu_driver_mock),
                                    decltype(notification_waiter),
                                    decltype(led_mock),
                                    decltype(logger)>
       imu_driver_executor{mpu_driver_mock,
                           notification_waiter,
                           led_mock,
                           logger,
                           period_ms};
};

TEST_F(ImuDriverExecutorTest, test_start_stop)
{
   EXPECT_CALL(mpu_driver_mock, start());
   imu_driver_executor.start();

   EXPECT_CALL(mpu_driver_mock, stop());
   imu_driver_executor.stop();
}

TEST_F(ImuDriverExecutorTest, test_execute_without_event)
{
   EXPECT_CALL(mpu_driver_mock, execute()).Times(0);
   EXPECT_CALL(mpu_driver_mock, notify_receive_complete()).Times(0);
   imu_driver_executor.run_once();
}

TEST_F(ImuDriverExecutorTest, test_execute_tick_event)
{
   aeromight_boundaries::ImuNotificationFlags flags{aeromight_boundaries::pos_to_value(aeromight_boundaries::ImuTaskEvents::tick)};
   EXPECT_CALL(notification_waiter, wait(period_ms)).WillOnce(testing::Return(flags));

   EXPECT_CALL(mpu_driver_mock, notify_receive_complete()).Times(0);
   EXPECT_CALL(mpu_driver_mock, execute());
   imu_driver_executor.run_once();
}

TEST_F(ImuDriverExecutorTest, test_execute_rx_complete_notification)
{
   aeromight_boundaries::ImuNotificationFlags flags{aeromight_boundaries::pos_to_value(aeromight_boundaries::ImuTaskEvents::rx_complete)};
   EXPECT_CALL(notification_waiter, wait(period_ms)).WillOnce(testing::Return(flags));

   EXPECT_CALL(mpu_driver_mock, notify_receive_complete());
   EXPECT_CALL(mpu_driver_mock, execute()).Times(0);
   imu_driver_executor.run_once();
}
