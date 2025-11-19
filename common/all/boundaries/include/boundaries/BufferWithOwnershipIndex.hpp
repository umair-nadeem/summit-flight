#pragma once

#include <cstdint>
#include <span>

namespace boundaries
{

template <typename T>
struct BufferWithOwnershipIndex
{
   std::span<const T> buffer{};
   std::size_t        index{};
};

}   // namespace boundaries
