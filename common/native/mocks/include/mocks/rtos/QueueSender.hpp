#pragma once

#include <gmock/gmock.h>

#include "interfaces/rtos/IQueueSender.hpp"

namespace mocks::rtos
{

template <typename T>
class QueueSenderNaggy
{
public:
   MOCK_METHOD(void, send_blocking, (const T), ());
   MOCK_METHOD(bool, send_if_possible, (const T), ());
   MOCK_METHOD(bool, send_from_isr, (const T, const bool), ());
};

static_assert(interfaces::rtos::IQueueSender<QueueSenderNaggy<int>, int>);

template <typename T>
using QueueSender = testing::NiceMock<QueueSenderNaggy<T>>;

}   // namespace mocks::rtos
