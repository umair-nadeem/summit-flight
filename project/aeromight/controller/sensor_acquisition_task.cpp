#include "SensorAcquisitionTaskData.h"
#include "aeromight_sensors/SensorAcquisition.hpp"
#include "hw/uart/uart.hpp"
#include "rtos/periodic_task.hpp"

extern "C"
{

   [[noreturn]] void sensor_acquisition_task(void* params)
   {
      assert(params != nullptr);
      auto* data = static_cast<controller::SensorAcquisitionTaskData*>(params);

      aeromight_sensors::SensorAcquisition<decltype(data->blue_led),
                                           decltype(data->logging_uart.transmitter)>
          sensor_acquisition{data->blue_led,
                             data->logging_uart.transmitter,
                             std::as_writable_bytes(std::span{data->logging_uart.dma_tx_buffer}),
                             controller::task::sensor_acq_task_period_in_ms};

      rtos::run_periodic_task(sensor_acquisition);
   }
}

extern "C"
{
   void USART1_IRQHandler()
   {
      auto& data = controller::sensor_acq_task_data;
      hw::uart::handle_uart_global_interrupt(data.logging_uart.config, data.logging_uart.isr_sem_giver);
   }

   void DMA2_Stream2_IRQHandler()
   {
      // do nothing, as rx is not applicable
   }
}
