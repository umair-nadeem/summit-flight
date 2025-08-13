#pragma once

#include <gmock/gmock.h>

#include "interfaces/hw/IUartTransmitter.hpp"

namespace mocks::hw
{

class UartTransmitterNaggy
{
public:
   MOCK_METHOD(void, send_blocking, (const uint32_t), ());
   MOCK_METHOD(void, send_and_return, (const uint32_t), ());
   MOCK_METHOD(std::span<std::byte>, get_buffer, (), (const));
};

static_assert(interfaces::hw::IUartTransmitter<UartTransmitterNaggy>);

using UartTransmitter = testing::NiceMock<UartTransmitterNaggy>;

}   // namespace mocks::hw
