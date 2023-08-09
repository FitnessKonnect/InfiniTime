#pragma once
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <components/heartrate/Ppg.h>
#include "drivers/Bma421.h"

#define CYCLE_DURATION       60 * 1000
#define MEASUREMENT_DURATION 5 * 1000
#define MEASUREMENT_DELAY    5

namespace Pinetime {
  namespace Drivers {
    class Hrs3300;
  }

  namespace Applications {
    class FkPpgTask {
    private:
      TaskHandle_t taskHandle;
      TimerHandle_t measurementTimer;
      TimerHandle_t waitTimer;

      QueueHandle_t messageQueue;
      States state = States::Running;
      bool measure = false;

      Drivers::Hrs3300& heartRateSensor;
      Pinetime::Drivers::Bma421& motionSensor;

      Controllers::Ppg ppg;

    public:
      enum class Messages : uint8_t { EnableMeasurement, DisableMeasurement, Toggle };
      enum class States { Stopped, Running, Measuring, Waiting };

      explicit HeartRateTask(Drivers::Hrs3300& heartRateSensor, Pinetime::Drivers::Bma421& motionSensor)
        : heartRateSensor {heartRateSensor}, motionSensor {motionSensor} {

        this->waitTimer = xTimerCreate("PPG-Wait", pdMS_TO_TICKS(CYCLE_DURATION), pdTRUE, this, FkPpgTask::StartMeasurement);
        this->measurementTimer =
          xTimerCreate("PPG-Measurement", pdMS_TO_TICKS(MEASUREMENT_DURATION), pdFalse, this, FkPpgTask::StopMeasurement);
      }

      int CurrentTaskDelay() {
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

      void Start() {
		SEGGER_RTT_printf(0, "starting ppg task\r\n");
        this->messageQueue = xQueueCreate(10, 1);
        if (pdPASS != xTaskCreate(FkPpgTask::Process, "FkPPG", 500, this, 0, &taskHandle)) {
		  SEGGER_RTT_printf(0, "failed to create ppg task\r\n");
          APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		  return;
        }
		SEGGER_RTT_printf(0, "created ppg task\r\n");
		this->EnableMeasurement();
      }

      static void Process(void* instance) {
        auto* app = static_cast<FkPpgTask*>(instance);
        app->Work();
      }

      void Work() {
        while (true) {
          auto delay = CurrentTaskDelay();
          Messages msg;
          auto result = xQueueReceive(messageQueue, &msg, delay);

          SEGGER_RTT_printf(0, "received message: %u\r\n", msg);
          if (result == pdTRUE) {
			switch (msg) {
			  case Messages::EnableMeasurement:
			  	SeggerRTT_printf(0, "msg=enable measurement\r\n");
            	EnableMeasurement();
				break;
			  case Messages::DisableMeasurement:
			  	SeggerRTT_printf(0, "msg=disable measurement\r\n");
				DisableMeasurement();
				break;
			  case Messages::Toggle:
				if (state == States::Stopped) {
				  EnableMeasurement();
				} else {
				  DisableMeasurement();
				}
				break;
			default:
				break;
          }
        }
      }

      void EnableMeasurement() {
        SEGGER_RTT_printf(0, "enabling measurement\r\n");
        this->measure = true;
        this->state = States::Running;
        xTimerStart(this->waitTimer, 0);
      }

      void DisableMeasurement() {
        SEGGER_RTT_printf(0, "disabling measurement\r\n");
        this->StopMeasurement();
        this->measure = false;
        this->state = States::Stopped;
        xTimerStop(this->waitTimer, 0);
        xTimerStop(this->measurementTimer, 0);
      }

      static void StartMeasurement() {
        auto* instance = pvTimerGetTimerID(xTimer);
        instance->state = States::Measuring;
        instance->measure = true;
        xTimerStart(instance->measurementTimer, 0);

        heartRateSensor.Enable();
        vTaskDelay(100);

        SEGGER_RTT_printf(0, "starting measurement\r\n");

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
        while (instance->measure) {
          auto hrs = heartRateSensor.ReadHrs();
          auto als = heartRateSensor.ReadAls();
          auto motionValues = motionSensor.Process();

          SEGGER_RTT_printf(0, "current hr, als, motion(xyz): %u, %u, %u, %u, %u\r\n", hrs, als, motionValues.x, motionValues.y, motionValues.z);

          vTaskDelay(5);
        }
      }

      static void StopMeasurement() {
        auto* instance = pvTimerGetTimerID(xTimer);
        SEGGER_RTT_printf(0, "stopping measurement\r\n");
        heartRateSensor.Disable();
        instance->measure = false;
        instance->state = States::Waiting;
        xTimerStop(this->measurementTimer, 0);
      }
    }
  }
}
