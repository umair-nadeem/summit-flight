#pragma once

#include <span>

#include "dshot/params.hpp"
#include "hw/dma/dma.hpp"
#include "hw/dshot/DshotConfig.hpp"
#include "hw/timer/timer.hpp"

namespace hw::dshot
{

template <std::size_t N>
class Dshot
{
   static constexpr float bit0_duty_cycle = 3.0f / 8.0f;
   static constexpr float bit1_duty_cycle = 3.0f / 4.0f;

   using DshotBuffers = std::array<std::array<uint16_t, ::dshot::frame_len>, N>;
   using DshotConfigs = std::array<hw::dshot::DshotConfig, N>;

public:
   explicit Dshot(DshotBuffers&       dshot_buffers,
                  const DshotConfigs& dshot_configs,
                  const uint16_t      timer_autoreload)
       : m_dshot_buffers{dshot_buffers},
         m_dshot_configs{dshot_configs},
         m_timer_autoreload{timer_autoreload},
         m_timing{static_cast<uint16_t>((timer_autoreload + 1u) * bit0_duty_cycle), static_cast<uint16_t>((timer_autoreload + 1u) * bit1_duty_cycle)}
   {
   }

   void prepare_for_communication()
   {
      for (const auto& config : m_dshot_configs)
      {
         LL_DMA_DisableStream(config.dma, config.stream);
      }

      for (const auto& config : m_dshot_configs)
      {
         while (LL_DMA_IsEnabledStream(config.dma, config.stream))
         {
         }
      }

      for (const auto& config : m_dshot_configs)
      {
         hw::dma::clear_dma_tc_flag(config.dma, config.stream);
         hw::dma::clear_dma_te_flag(config.dma, config.stream);
         hw::dma::clear_dma_ht_flag(config.dma, config.stream);
      }

      for (std::size_t i = 0; i < N; i++)
      {
         const auto& config = m_dshot_configs[i];

         LL_DMA_SetPeriphAddress(config.dma, config.stream, reinterpret_cast<uint32_t>(config.ccr));
         LL_DMA_SetMemoryAddress(config.dma, config.stream, reinterpret_cast<uint32_t>(m_dshot_buffers[i].data()));
      }

      for (const auto& config : m_dshot_configs)
      {
         ::hw::timer::enable_timer_dma(config.timer_handle, config.timer_channel);
      }
   }

   void send(const std::array<uint16_t, N>& frames)
   {
      for (std::size_t i = 0; i < N; i++)
      {
         fill_buffer(m_dshot_buffers[i], frames[i]);
      }

      for (const auto& config : m_dshot_configs)
      {
         LL_DMA_DisableStream(config.dma, config.stream);
      }

      for (const auto& config : m_dshot_configs)
      {
         while (LL_DMA_IsEnabledStream(config.dma, config.stream))
         {
         }
      }

      for (const auto& config : m_dshot_configs)
      {
         hw::dma::clear_dma_tc_flag(config.dma, config.stream);
         hw::dma::clear_dma_te_flag(config.dma, config.stream);
         hw::dma::clear_dma_ht_flag(config.dma, config.stream);
      }

      for (std::size_t i = 0; i < N; i++)
      {
         const auto& config = m_dshot_configs[i];

         LL_DMA_SetMemoryAddress(config.dma, config.stream, reinterpret_cast<uint32_t>(m_dshot_buffers[i].data()));
      }

      for (const auto& config : m_dshot_configs)
      {
         LL_DMA_SetDataLength(config.dma, config.stream, ::dshot::frame_len);
      }

      for (const auto& config : m_dshot_configs)
      {
         LL_DMA_EnableStream(config.dma, config.stream);
      }
   }

private:
   void fill_buffer(std::span<uint16_t> buffer, const uint16_t frame) const noexcept
   {
      for (uint8_t i = 0; i < ::dshot::frame_bits; i++)
      {
         const bool bit = ((frame & static_cast<uint16_t>(1u << (15u - i))) != 0u);
         buffer[i]      = bit ? m_timing.ccr_one : m_timing.ccr_zero;
      }

      buffer[16u] = 0;
      buffer[17u] = 0;
   }

   struct Timing
   {
      uint16_t ccr_zero;
      uint16_t ccr_one;
   };

   DshotBuffers&       m_dshot_buffers;
   const DshotConfigs& m_dshot_configs;
   const uint16_t      m_timer_autoreload;
   const Timing        m_timing;
};

}   // namespace hw::dshot
