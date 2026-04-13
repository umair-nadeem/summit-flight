#pragma once

#include <stdexcept>

#include "interfaces/rtos/INotifier.hpp"

namespace mocks::rtos
{

class NotifierNaggy
{
public:
   MOCK_METHOD(void, notify, (), ());
   MOCK_METHOD(void, notify_from_isr, (), ());
};

static_assert(interfaces::rtos::INotifier<NotifierNaggy>);

using Notifier = testing::NiceMock<NotifierNaggy>;

}   // namespace mocks::rtos
