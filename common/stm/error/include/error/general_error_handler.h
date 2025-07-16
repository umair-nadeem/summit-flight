#pragma once

#include <source_location>

namespace error
{

void assert(const bool condition, const std::source_location location = std::source_location::current());

}   // namespace error
