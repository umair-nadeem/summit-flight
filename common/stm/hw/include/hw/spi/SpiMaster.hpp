#pragma once

#include <span>

#include "error/error_handler.hpp"
#include "hw/spi/SpiConfig.hpp"
#include "interfaces/pcb_component/IEnabler.hpp"

namespace hw::spi
{

template <interfaces::pcb_component::IEnabler ChipSelect, typename TimeoutEvaluator>
class SpiMaster
{
public:
   explicit SpiMaster(SpiConfig& spi_config, ChipSelect& chip_select, TimeoutEvaluator& timeout_evaluator)
       : m_spi_config{spi_config},
         m_chip_select{chip_select},
         m_timeout_evaluator{timeout_evaluator}
   {
   }

   void prepare_for_communication()
   {
      // enable spi
      LL_SPI_Enable(m_spi_config.spi_handle);
   }

   bool transfer8(std::span<const uint8_t> tx_buffer,
                  std::span<uint8_t>       rx_buffer,
                  const std::size_t        data_count,
                  const uint32_t           timeout_us)
   {
      error::verify(tx_buffer.data() != rx_buffer.data());
      error::verify(tx_buffer.size() == rx_buffer.size());
      error::verify((0 < data_count) && (data_count <= tx_buffer.size()));

      std::size_t tx_count = data_count;
      std::size_t rx_count = data_count;

      const bool use_timeout = (0 < timeout_us);
      if (use_timeout)
      {
         m_timeout_evaluator.set_timeout(timeout_us);
         m_timeout_evaluator.start_measurement();
      }

      enable_chip_select();

      while ((tx_count > 0) || (rx_count > 0))
      {
         // @TODO: check tx flag and send byte
         if ((tx_count > 0) && (LL_SPI_IsActiveFlag_TXE(m_spi_config.spi_handle) == 1))
         {
            LL_SPI_TransmitData8(m_spi_config.spi_handle, tx_buffer[data_count - tx_count]);
            tx_count--;
         }

         if ((rx_count > 0) && (LL_SPI_IsActiveFlag_RXNE(m_spi_config.spi_handle) == 1))
         {
            rx_buffer[data_count - rx_count] = LL_SPI_ReceiveData8(m_spi_config.spi_handle);
            rx_count--;
         }

         if (use_timeout && m_timeout_evaluator.timeout_occurred())
         {
            disable_chip_select();
            return false;
         }
      }

      while (LL_SPI_IsActiveFlag_BSY(m_spi_config.spi_handle))
      {
         if (use_timeout && m_timeout_evaluator.timeout_occurred())
         {
            disable_chip_select();
            return false;
         }
      }

      disable_chip_select();
      return true;
   }

   // void spi2_test()
   // {
   //    // enable SPI2
   //    LL_SPI_Enable(SPI2);

   //    enable_chip_select();

   //    // wait until TXE=1
   //    while (!LL_SPI_IsActiveFlag_TXE(SPI2))
   //    {
   //    }

   //    // write one byte
   //    LL_SPI_TransmitData8(SPI2, 0x9A);

   //    while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
   //    {
   //    }

   //    LL_SPI_ReceiveData8(SPI2);

   //    // wait until BSY=0 (shifting finished)
   //    while (LL_SPI_IsActiveFlag_BSY(SPI2))
   //    {
   //    }

   //    // pull CS high
   //    disable_chip_select();
   // }

private:
   void enable_chip_select()
   {
      m_chip_select.enable();
   }

   void disable_chip_select()
   {
      m_chip_select.disable();
   }

   SpiConfig&        m_spi_config;
   ChipSelect&       m_chip_select;
   TimeoutEvaluator& m_timeout_evaluator;
};

}   // namespace hw::spi
