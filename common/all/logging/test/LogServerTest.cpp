#include "logging/LogServer.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "logging/log_params.hpp"
#include "mocks/rtos/QueueReceiver.hpp"

class LogChannelMock
{
public:
   MOCK_METHOD(void, publish_log, (std::span<const char>), ());
};

class LogServerTest : public testing::Test
{
protected:
   mocks::rtos::QueueReceiver<logging::params::LogBuffer>                        queue_receiver_mock{};
   LogChannelMock                                                                log_channel_mock{};
   logging::LogServer<decltype(queue_receiver_mock), decltype(log_channel_mock)> log_server{queue_receiver_mock,
                                                                                            log_channel_mock};
};

TEST_F(LogServerTest, throw_exception_without_returning_queue_item)
{
   EXPECT_THROW(log_server.run_once(), std::runtime_error);
}

TEST_F(LogServerTest, publish_single_print)
{
   logging::params::LogBuffer test_print{"this is a test log\n"};

   EXPECT_CALL(queue_receiver_mock, receive_blocking).WillOnce(testing::Return(test_print));

   EXPECT_CALL(log_channel_mock, publish_log(testing::ElementsAreArray(test_print.begin(), test_print.end())));

   log_server.run_once();
}
