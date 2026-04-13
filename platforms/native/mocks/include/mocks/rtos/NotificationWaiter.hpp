#pragma once

#include <stdexcept>

#include "interfaces/rtos/INotificationWaiter.hpp"

namespace mocks::rtos
{

template <typename T>
class NotificationWaiterNaggy
{
public:
   MOCK_METHOD(std::optional<T>, wait, (std::size_t), ());
};

static_assert(interfaces::rtos::INotificationWaiter<NotificationWaiterNaggy<int>, int>);

template <typename T>
using NotificationWaiter = testing::NiceMock<NotificationWaiterNaggy<T>>;

}   // namespace mocks::rtos
