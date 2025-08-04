#pragma once

namespace interfaces
{

template <typename T>
concept ILogChannel = requires(T t, std::span<const char> msg) {
   {
      t.publish_log(msg)
   }
   -> std::same_as<void>;
};

}   // namespace interfaces
