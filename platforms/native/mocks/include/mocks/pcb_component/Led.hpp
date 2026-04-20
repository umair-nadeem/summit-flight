#pragma once

#include <gmock/gmock.h>

#include "interfaces/pcb_component/ILed.hpp"

namespace mocks::pcb_component
{

class Led
{
public:
   MOCK_METHOD(void, turn_on, (), ());
   MOCK_METHOD(void, turn_off, (), ());
   MOCK_METHOD(void, toggle, (const uint32_t), ());
};

static_assert(interfaces::pcb_component::ILed<Led>);

}   // namespace mocks::pcb_component
