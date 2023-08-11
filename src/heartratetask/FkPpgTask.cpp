#ifndef __FKPPGTASK_CPP__
#define __FKPPGTASK_CPP__
#include "heartratetask/FkPpgTask.h"
#include <task.h>
#include "SEGGER_RTT.h"

using namespace Pinetime::Applications;

#define CYCLE_DURATION       60 * 1000
#define MEASUREMENT_DURATION 5 * 1000
#define MEASUREMENT_DELAY    5

FkPpgTask::FkPpgTask(Drivers::Hrs3300& heartRateSensor, Pinetime::Drivers::Bma421& motionSensor) 
  : heartRateSensor(heartRateSensor), motionSensor(motionSensor) {

  this->waitTimer = xTimerCreate("PPG-Wait", pdMS_TO_TICKS(CYCLE_DURATION), pdTRUE, this, FkPpgTask::StartMeasurementFK);
  this->measurementTimer = xTimerCreate("PPG-Measurement", pdMS_TO_TICKS(MEASUREMENT_DURATION), pdFALSE, this, FkPpgTask::StopMeasurementFK);
}

int FkPpgTask::CurrentTaskDelayFK() {
  switch (state) {
    case States::Measuring:
      return ppg.deltaTms;
    case States::Running:
      return MEASUREMENT_DURATION;
    case States::Waiting:
      return CYCLE_DURATION;
    default:
      return portMAX_DELAY;
  }
}

void FkPpgTask::StartFK() {
  SEGGER_RTT_printf(0, "starting ppg task\r\n");
  this->messageQueue = xQueueCreate(10, 1);
  if (pdPASS != xTaskCreate(FkPpgTask::ProcessFK, "FkPPG", 500, this, 0, &taskHandle)) {
    SEGGER_RTT_printf(0, "failed to create ppg task\r\n");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    return;
  }
  SEGGER_RTT_printf(0, "created ppg task\r\n");
  this->EnableMeasurementFK();
}

void FkPpgTask::ProcessFK(void* instance) {
  auto* app = static_cast<FkPpgTask*>(instance);
  app->WorkFK();
}

void FkPpgTask::WorkFK() {
  while (true) {
    auto delay = CurrentTaskDelayFK();
    Messages msg;
    auto result = xQueueReceive(messageQueue, &msg, delay);

    SEGGER_RTT_printf(0, "received message: %u\r\n", msg);
    if (result == pdTRUE) {
      switch (msg) {
        case Messages::EnableMeasurement:
          SEGGER_RTT_printf(0, "msg=enable measurement\r\n");
          EnableMeasurementFK();
          break;
        case Messages::DisableMeasurement:
          SEGGER_RTT_printf(0, "msg=disable measurement\r\n");
          DisableMeasurementFK();
          break;
        case Messages::Toggle:
          if (this->state == States::Stopped) {
            EnableMeasurementFK();
          } else {
            DisableMeasurementFK();
          }
          break;
        default:
          break;
      }
    }
  }
}

void FkPpgTask::EnableMeasurementFK() {
  SEGGER_RTT_printf(0, "enabling measurement\r\n");
  this->measure = true;
  this->state = States::Running;
  xTimerStart(this->waitTimer, 0);
}

void FkPpgTask::DisableMeasurementFK() {
  SEGGER_RTT_printf(0, "disabling measurement\r\n");
  this->heartRateSensor.Disable();
  this->measure = false;
  this->state = States::Stopped;
  xTimerStop(this->waitTimer, 0);
  xTimerStop(this->measurementTimer, 0);
}

void FkPpgTask::StartMeasurementFK(TimerHandle_t xTimer) {
  auto* instance = static_cast<FkPpgTask*>(pvTimerGetTimerID(xTimer));
  instance->state = States::Measuring;
  instance->measure = true;
  xTimerStart(instance->measurementTimer, 0);

  instance->heartRateSensor.Enable();
  vTaskDelay(100);

  SEGGER_RTT_printf(0, "starting measurement\r\n");

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (instance->measure) {
    auto hrs = instance->heartRateSensor.ReadHrs();
    auto als = instance->heartRateSensor.ReadAls();
    auto motionValues = instance->motionSensor.Process();

    SEGGER_RTT_printf(0, "current hr, als, motion(xyz): %u, %u, %u, %u, %u\r\n", hrs, als, motionValues.x, motionValues.y, motionValues.z);

    vTaskDelay(5);
  }
}

void FkPpgTask::StopMeasurementFK(TimerHandle_t xTimer) {
  auto* instance = static_cast<FkPpgTask*>(pvTimerGetTimerID(xTimer));
  SEGGER_RTT_printf(0, "stopping measurement\r\n");
  instance->heartRateSensor.Disable();
  instance->measure = false;
  instance->state = States::Waiting;
  xTimerStop(instance->measurementTimer, 0);
}

void FkPpgTask::PushMessageFK(FkPpgTask::Messages msg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(messageQueue, &msg, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    /* Actual macro used here is port specific. */
    // TODO : should I do something here?
  }
}
#endif //__FKPPGTASK_CPP__