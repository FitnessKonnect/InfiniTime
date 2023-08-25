#pragma once
#ifndef __FKPPGTASK_H__
  #define __FKPPGTASK_H__
  #include <FreeRTOS.h>
  #include <task.h>
  #include <queue.h>
  #include <components/heartrate/Ppg.h>
  #include "drivers/Bma421.h"
  #include <drivers/Hrs3300.h>
  #include "heartratetask/HeartRateTask.h"

  #include <timers.h>

  #define CYCLE_DURATION       15 * 1000
  #define MEASUREMENT_DURATION 5 * 1000
  #define MEASUREMENT_DELAY    5

namespace Pinetime {
  namespace Drivers {
    class Hrs3300;
    class Bma421;
  }

  namespace Controllers {
	class FS;
  }

  namespace Applications {
    class HeartRateTask;

    class FkPpgTask {
    private:
      TaskHandle_t taskHandle;
      TimerHandle_t measurementTimer;
      TimerHandle_t waitTimer;

      bool measure = false;

      Drivers::Hrs3300& heartRateSensor;
      Pinetime::Drivers::Bma421& motionSensor;

      Controllers::Ppg ppg;
      HeartRateTask* heartRateTask;

    public:
      enum class Messages : uint8_t { EnableMeasurement, DisableMeasurement, Toggle };
      enum class States { Stopped, Running, Measuring, Waiting };
      States state = States::Running;

      explicit FkPpgTask(Drivers::Hrs3300& heartRateSensor, Drivers::Bma421& motionSensor);

      void StartFK(HeartRateTask& heartRateTask, Pinetime::Controllers::FS& fs);

      void EnableMeasurementFK();

      void DisableMeasurementFK();

	  void listMatchingFiles(Pinetime::Controllers::FS& fs);

      static void StartMeasurementFK(TimerHandle_t xTimer);

      static void StopMeasurementFK(TimerHandle_t xTimer);
    };
  }
}
#endif //__FKPPGTASK_H__
