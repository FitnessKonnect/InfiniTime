#include "heartratetask/HeartRateTask.h"
#include <drivers/Hrs3300.h>
#include <components/heartrate/HeartRateController.h>
#include <nrf_log.h>
#include "SEGGER_RTT.h"

using namespace Pinetime::Applications;

HeartRateTask::HeartRateTask(Drivers::Hrs3300& heartRateSensor, Controllers::HeartRateController& controller)
  : heartRateSensor {heartRateSensor}, controller {controller} {
}

void HeartRateTask::Start() {
  messageQueue = xQueueCreate(10, 1);
  controller.SetHeartRateTask(this);

  if (pdPASS != xTaskCreate(HeartRateTask::Process, "Heartrate", 500, this, 0, &taskHandle)) {
    SEGGER_RTT_printf(0, "Failed to create HeartRateTask\r\n");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

void HeartRateTask::Process(void* instance) {
  auto* app = static_cast<HeartRateTask*>(instance);
  app->Work();
}

void HeartRateTask::Work() {
  while (true) {
    Messages msg;
    uint32_t delay;
    if (state == States::Running) {
      if (measurementStarted) {
        delay = ppg.deltaTms;
      } else {
        delay = 100;
      }
    } else {
      delay = portMAX_DELAY;
    }

    if (xQueueReceive(messageQueue, &msg, delay)) {
      switch (msg) {
        case Messages::GoToSleep:
          StopMeasurement();
          state = States::Idle;
          break;
        case Messages::WakeUp:
          state = States::Running;
          if (measurementStarted) {
            StartMeasurement();
          }
          break;
        case Messages::StartMeasurement:
          if (measurementStarted) {
            break;
          }
          StartMeasurement();
          measurementStarted = true;
          break;
        case Messages::StopMeasurement:
          if (!measurementStarted) {
            break;
          }
          StopMeasurement();
          measurementStarted = false;
          break;
      }
    }

    if (measurementStarted) {
      auto hrs = heartRateSensor.ReadHrs();
      auto als = heartRateSensor.ReadAls();

      SEGGER_RTT_printf(0, "current hr, als: %u, %u\r\n", hrs, als);
    }
  }
}

void HeartRateTask::PushMessage(HeartRateTask::Messages msg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(messageQueue, &msg, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    /* Actual macro used here is port specific. */
    // TODO : should I do something here?
  }
}

void HeartRateTask::StartMeasurement() {
  heartRateSensor.Enable();
  ppg.Reset(true);
  vTaskDelay(100);
}

void HeartRateTask::StopMeasurement() {
  heartRateSensor.Disable();
  ppg.Reset(true);
  vTaskDelay(100);
}
