#pragma once

#include <stdexcept>

#include "interfaces/rtos/IQueueReceiver.hpp"

namespace mocks::rtos
{

template <typename T>
class QueueReceiverNaggy
{
public:
   QueueReceiverNaggy()
   {
      ON_CALL(*this, receive_blocking).WillByDefault(testing::Throw(std::runtime_error("error")));
   }

   MOCK_METHOD(T, receive_blocking, (), ());
   MOCK_METHOD(std::optional<T>, receive_if_available, (), ());
   MOCK_METHOD(std::optional<T>, receive_latest, (), ());
   MOCK_METHOD(bool, receive_from_isr, (T&), ());
};

static_assert(interfaces::rtos::IQueueReceiver<QueueReceiverNaggy<int>, int>);

template <typename T>
using QueueReceiver = testing::NiceMock<QueueReceiverNaggy<T>>;

}   // namespace mocks::rtos
