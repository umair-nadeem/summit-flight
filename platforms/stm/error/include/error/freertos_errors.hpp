#pragma once

#include <source_location>

namespace error
{

void freertos_assert(const bool result, const std::source_location location = std::source_location::current());

}   // namespace error
