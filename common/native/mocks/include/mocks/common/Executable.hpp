#pragma once

#include <gmock/gmock.h>

namespace mocks::common
{

class Executable
{
public:
   MOCK_METHOD(void, execute, (), ());
};

}   // namespace mocks::common
