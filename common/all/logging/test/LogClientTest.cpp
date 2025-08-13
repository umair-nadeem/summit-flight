#include "logging/LogClient.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "logging/log_params.hpp"
#include "mocks/rtos/QueueSender.hpp"

class LogClientTest : public testing::Test
{
protected:
   const char* const                                    logger_name{"log_test"};
   mocks::rtos::QueueSender<logging::params::LogBuffer> queue_sender_mock{};
   logging::LogClient<decltype(queue_sender_mock)>      log_client{queue_sender_mock, logger_name};
};

TEST_F(LogClientTest, print_without_enabling)
{
   EXPECT_CALL(queue_sender_mock, send_blocking).Times(0);
   EXPECT_CALL(queue_sender_mock, send_if_possible).Times(0);
   EXPECT_CALL(queue_sender_mock, send_from_isr).Times(0);

   log_client.print("this will not print");
}

TEST_F(LogClientTest, print_empty_log)
{
   log_client.enable();
   logging::params::LogBuffer expected_log{"log_test: \n"};

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.print("");

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.printf("%s", "");
}

TEST_F(LogClientTest, print_simple_msg)
{
   log_client.enable();
   logging::params::LogBuffer expected_log{"log_test: this is A Test Log!\n"};

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.print("this is A Test Log!");

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.printf("%s", "this is A Test Log!");
}

TEST_F(LogClientTest, print_random_numbers)
{
   log_client.enable();
   logging::params::LogBuffer expected_log{"log_test: here are some numbers: 4 + 0xd = 17\n"};

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.print("here are some numbers: 4 + 0xd = 17");

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.printf("here are some numbers: %u + 0x%x = %d", 4, 13, 17);
}

TEST_F(LogClientTest, cut_short_oversized_msg)
{
   log_client.enable();
   logging::params::LogBuffer expected_log{"log_test: This msg is longer than buffer length of 64 chars wi\n"};

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.print("This msg is longer than buffer length of 64 chars with as many unique words possible. LogClient should cut it short to accommodate the logger name and buffer length.");

   EXPECT_CALL(queue_sender_mock, send_if_possible(expected_log));
   log_client.printf("%s", "This msg is longer than buffer length of 64 chars with as many unique words possible. LogClient should cut it short to accommodate the logger name and buffer length.");
}
