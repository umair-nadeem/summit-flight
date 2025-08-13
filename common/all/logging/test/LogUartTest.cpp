#include "logging/LogUart.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "error/AssertFailureException.hpp"
#include "error/error_handler.hpp"
#include "logging/log_params.hpp"
#include "mocks/hw/UartTransmitter.hpp"

extern "C"
{
   void putchar_([[maybe_unused]] char c)
   {
      error::stop_operation();
   }
}   // extern "C"

class LogUartTest : public testing::Test
{
protected:
   std::array<std::byte, logging::params::max_log_len> uart_tx_buffer{};
   mocks::hw::UartTransmitter                          uart_transmitter_mock{};
   logging::LogUart<decltype(uart_transmitter_mock)>   log_uart{uart_transmitter_mock};
};

TEST_F(LogUartTest, exception_is_thrown_with_undersized_buffer)
{
   logging::params::LogBuffer test_print{"This is a print"};

   EXPECT_CALL(uart_transmitter_mock, get_buffer()).WillOnce(testing::Return(std::span{uart_tx_buffer.data(), test_print.size() - 1}));

   EXPECT_THROW(log_uart.publish_log(test_print), error::AssertFailureException);
}

TEST_F(LogUartTest, log_without_line_feed)
{
   logging::params::LogBuffer test_print{};
   test_print.fill('t');

   EXPECT_CALL(uart_transmitter_mock, get_buffer()).WillOnce(testing::Return(std::span{uart_tx_buffer}));

   EXPECT_CALL(uart_transmitter_mock, send_and_return(logging::params::max_log_len));

   log_uart.publish_log(test_print);

   std::span<std::byte> test_print_bytes = std::as_writable_bytes(std::span{test_print});
   EXPECT_THAT(uart_tx_buffer, testing::ElementsAreArray(test_print_bytes.begin(), test_print_bytes.end()));
}

TEST_F(LogUartTest, log_with_line_feed)
{
   logging::params::LogBuffer test_print{"Test print with line feed char\n"};

   EXPECT_CALL(uart_transmitter_mock, get_buffer()).WillOnce(testing::Return(std::span{uart_tx_buffer}));

   EXPECT_CALL(uart_transmitter_mock, send_and_return(static_cast<uint32_t>(strlen(test_print.data()))));

   log_uart.publish_log(test_print);

   std::span<std::byte> test_print_bytes = std::as_writable_bytes(std::span{test_print});
   EXPECT_THAT(uart_tx_buffer, testing::ElementsAreArray(test_print_bytes.begin(), test_print_bytes.end()));
}
