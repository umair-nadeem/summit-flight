#include "ControlTaskData.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/Estimation.hpp"
#include "aeromight_control/EstimationAndControl.hpp"
#include "error/error_handler.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{

   [[noreturn]] void control_task(void* const params)
   {
      error::verify(params != nullptr);
      // auto* data = static_cast<controller::ControlTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      LogClient logger_estimation{logging::logging_queue_sender, "estimation"};
      LogClient logger_control{logging::logging_queue_sender, "control"};

      aeromight_control::Estimation<LogClient> estimation{logger_estimation};
      aeromight_control::Control<LogClient>    control{logger_control};

      aeromight_control::EstimationAndControl<decltype(estimation),
                                              decltype(control)>
          estimation_and_control{estimation,
                                 control,
                                 controller::task::control_task_period_in_ms};

      estimation_and_control.start();
      rtos::run_periodic_task(estimation_and_control);
   }

}   // extern "C"
