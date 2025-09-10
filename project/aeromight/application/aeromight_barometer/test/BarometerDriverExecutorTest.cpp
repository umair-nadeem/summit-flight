#include "aeromight_barometer/BarometerDriverExecutor.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/common/Executable.hpp"
#include "mocks/rtos/NotificationWaiter.hpp"

class BarometerDriverExecutorTest : public testing::Test
{
protected:
   mocks::common::Executable                                                         baro_driver_mock{};
   mocks::rtos::NotificationWaiter<aeromight_boundaries::BarometerNotificationFlags> notification_waiter{};
   std::size_t                                                                       period_ms{40u};
   std::size_t                                                                       notification_wait_period_in_ms{10u};

   aeromight_barometer::BarometerDriverExecutor<decltype(baro_driver_mock),
                                                decltype(notification_waiter)>
       barometer_driver_executor{baro_driver_mock,
                                 notification_waiter,
                                 period_ms,
                                 notification_wait_period_in_ms};
};

TEST_F(BarometerDriverExecutorTest, test_start_stop)
{
   EXPECT_CALL(baro_driver_mock, start());
   barometer_driver_executor.start();

   EXPECT_CALL(baro_driver_mock, stop());
   barometer_driver_executor.stop();
}

TEST_F(BarometerDriverExecutorTest, test_execute_without_event)
{
   EXPECT_CALL(baro_driver_mock, execute());
   EXPECT_CALL(baro_driver_mock, notify_receive_complete()).Times(0);
   barometer_driver_executor.run_once();
}

TEST_F(BarometerDriverExecutorTest, test_execute_rx_complete_notification)
{
   aeromight_boundaries::BarometerNotificationFlags flags{aeromight_boundaries::pos_to_value(aeromight_boundaries::BarometerTaskEvents::rx_complete)};
   EXPECT_CALL(notification_waiter, wait(notification_wait_period_in_ms)).WillOnce(testing::Return(flags));

   EXPECT_CALL(baro_driver_mock, notify_receive_complete());
   EXPECT_CALL(baro_driver_mock, execute());
   barometer_driver_executor.run_once();
}
