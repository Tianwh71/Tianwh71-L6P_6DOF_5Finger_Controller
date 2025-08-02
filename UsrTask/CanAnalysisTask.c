#include "CanAnalysisTask.h"
#include "upper_comm_protocol_fml.h"

void StartCanAnalysisTask(void *argument)
{

  /* Infinite loop */
  for (;;)
  {
    osMessageQueueGet(Can_RX_QueueHandle, &can_rx_get_queue, 0, (TickType_t)portMAX_DELAY);
    event_can_tip_sensor_dispose();
    uint32_t count = osMessageQueueGetCount(Can_RX_QueueHandle);
		osDelay(1);
  }
}
