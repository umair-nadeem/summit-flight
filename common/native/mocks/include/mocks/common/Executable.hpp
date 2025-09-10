#pragma once

#include <gmock/gmock.h>

namespace mocks::common
{

class Executable
{
public:
   MOCK_METHOD(void, execute, (), ());
   MOCK_METHOD(void, start, (), ());
   MOCK_METHOD(void, stop, (), ());
   MOCK_METHOD(void, notify_receive_complete, (), ());
};

}   // namespace mocks::common
