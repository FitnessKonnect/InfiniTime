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
	SEGGER_RTT_printf(0, "HRT: Failed to create task\r\n");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

void HeartRateTask::Process(void* instance) {
  auto* app = static_cast<HeartRateTask*>(instance);
  app->Work();
}

void HeartRateTask::Work() {
  int lastBpm = 0;

  while (true) {
    auto delay = CurrentTaskDelay();
    Messages msg;

    auto result = xQueueReceive(messageQueue, &msg, delay);
    const char* msgStr = "";
    switch (msg) {
      case Messages::GoToSleep:
        msgStr = "GoToSleep";
        break;
      case Messages::WakeUp:
        msgStr = "WakeUp";
        break;
      case Messages::StartMeasurement:
        msgStr = "StartMeasurement";
        break;
      case Messages::StopMeasurement:
        msgStr = "StopMeasurement";
        break;
    }
    const char* stateStr = "";
    switch (state) {
      case States::Idle:
        stateStr = "Idle";
        break;
      case States::Running:
        stateStr = "Running";
        break;
      case States::Measuring:
        stateStr = "Measuring";
        break;
      case States::BackgroundMeasuring:
        stateStr = "BackgroundMeasuring";
        break;
      case States::BackgroundWaiting:
        stateStr = "BackgroundWaiting";
        break;
    }
    if (result == pdTRUE) {
      SEGGER_RTT_printf(0, "HRT message = %s, result = %s\r\n", msgStr, result == pdTRUE ? "pdTRUE" : "pdFALSE");
      switch (msg) {
        case Messages::GoToSleep:
          SEGGER_RTT_printf(0, "Goodnight ... %s\r\n", stateStr);
          if (state == States::Running) {
            state = States::Idle;
          } else if (state == States::Measuring) {
            state = States::BackgroundWaiting;
            StartWaiting();
          }
          StopMeasurement();
          break;
        case Messages::WakeUp:
          SEGGER_RTT_printf(0, "I am waking up to ash and dust .... %s\r\n", stateStr);
          if (state == States::Idle) {
            state = States::Running;
            lastBpm = 0;
            StartMeasurement();
          } else if (state == States::BackgroundMeasuring) {
            state = States::Measuring;
          } else if (state == States::BackgroundWaiting) {
            state = States::Measuring;
            StartMeasurement();
          } else if (state == States::Running) {
            state = States::Measuring; // this fucked us as the sensor no longer starts ... the sensor was starting only when State=RUNNING
            lastBpm = 0;
            StartMeasurement();
          }
          break;
        case Messages::StartMeasurement:
          if (state == States::Measuring || state == States::BackgroundMeasuring) {
            break;
          }
          state = States::Measuring;
          lastBpm = 0;
          StartMeasurement();
          break;
        case Messages::StopMeasurement:
          if (state == States::Running || state == States::Idle) {
            break;
          }
          if (state == States::Measuring) {
            state = States::Running;
          } else if (state == States::BackgroundMeasuring) {
            state = States::Idle;
          }
          StopMeasurement();
          break;
      }
    }

    if (state == States::BackgroundWaiting) {
      HandleBackgroundWaiting();
    } else if (state == States::BackgroundMeasuring || state == States::Measuring) {
      HandleSensorData(&lastBpm);
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
  measurementStart = xTaskGetTickCount();
}

void HeartRateTask::StopMeasurement() {
  heartRateSensor.Disable();
  ppg.Reset(true);
  vTaskDelay(100);
}

void HeartRateTask::StartWaiting() {
  StopMeasurement();
  backgroundWaitingStart = xTaskGetTickCount();
}

void HeartRateTask::HandleBackgroundWaiting() {
  if (xTaskGetTickCount() - backgroundWaitingStart >= DURATION_BETWEEN_BACKGROUND_MEASUREMENTS) {
    state = States::BackgroundMeasuring;
    StartMeasurement();
  }
}

void HeartRateTask::HandleSensorData(int* lastBpm) {
  int8_t ambient = ppg.Preprocess(heartRateSensor.ReadHrs(), heartRateSensor.ReadAls());
  int bpm = ppg.HeartRate();

  // If ambient light detected or a reset requested (bpm < 0)
  if (ambient > 0) {
    // Reset all DAQ buffers
    ppg.Reset(true);
  } else if (bpm < 0) {
    // Reset all DAQ buffers except HRS buffer
    ppg.Reset(false);
    // Set HR to zero and update
    bpm = 0;
  }

  if (*lastBpm == 0 && bpm == 0) {
    controller.Update(Controllers::HeartRateController::States::NotEnoughData, bpm);
  }

  if (bpm != 0) {
    *lastBpm = bpm;
    controller.Update(Controllers::HeartRateController::States::Running, bpm);
    if (state == States::BackgroundMeasuring) {
      state = States::BackgroundWaiting;
      StartWaiting();
    }
  }
  if (bpm == 0 && state == States::BackgroundMeasuring &&
      xTaskGetTickCount() - measurementStart >= DURATION_UNTIL_BACKGROUND_MEASURMENT_IS_STOPPED) {
    state = States::BackgroundWaiting;
    StartWaiting();
  }
}

int HeartRateTask::CurrentTaskDelay() {
  switch (state) {
    case States::Measuring:
    case States::BackgroundMeasuring:
      return ppg.deltaTms;
    case States::Running:
      return 100;
    case States::BackgroundWaiting:
      return 10000;
    default:
      return portMAX_DELAY;
  }
}

;

void HeartRateTask::ReadAndPrintPpgData(int seconds, int delayInSeconds, Pinetime::Drivers::Bma421& motionSensor) {
  // Convert seconds and delayInSeconds to ticks as vTaskDelay function uses ticks for delay
  // Assuming tick rate is 1000 Hz, so 1 tick = 1 ms
  TickType_t secondsInTicks = seconds * 1000;

  while (true) {
    heartRateSensor.Enable();
    vTaskDelay(100);

    TickType_t startTick = xTaskGetTickCount();

    SEGGER_RTT_printf(0, "startTick: %u\n", startTick);
    while ((xTaskGetTickCount() - startTick) < secondsInTicks) { // Loop for 'seconds' duration

      // Print the length of the PPG data
      SEGGER_RTT_printf(0, "current hr: %u\n", heartRateSensor.ReadHrs());
      SEGGER_RTT_printf(0, "current ambient light: %u\n", heartRateSensor.ReadAls());

      auto motionValues = motionSensor.Process();
      SEGGER_RTT_printf(0, " X: %d, Y: %d, Z: %d\n", motionValues.x, motionValues.y, motionValues.z);

    }

    heartRateSensor.Disable();
    vTaskDelay(delayInSeconds * 1000);
  }
}
