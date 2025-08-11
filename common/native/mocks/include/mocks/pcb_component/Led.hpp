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
};

static_assert(interfaces::pcb_component::ILed<Led>);

}   // namespace mocks::pcb_component
