#pragma once

namespace interfaces
{

template <typename C>
concept ILogChannel = requires(C c, std::span<const char> msg) {
   {
      c.publish_log(msg)
   }
   -> std::same_as<void>;
};

}   // namespace interfaces
