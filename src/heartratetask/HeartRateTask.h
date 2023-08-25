#pragma once
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <components/heartrate/Ppg.h>

#define MAX_PPG_DATA_SIZE 100

namespace Pinetime {
  namespace System {
    class SystemTask;
  }

  namespace Drivers {
    class Hrs3300;
    class Bma421;
  }

  namespace Controllers {
    class HeartRateController;
    class DateTime;
    class FS;
  }

  namespace Applications {
    struct PPG_Data {
      uint32_t hrs;
      uint32_t als;
      int16_t motion_x;
      int16_t motion_y;
      int16_t motion_z;
    };

    class HeartRateTask {
    public:
      enum class Messages : uint8_t { GoToSleep, WakeUp, StartMeasurement, StopMeasurement };
      enum class States { Idle, Running };

      explicit HeartRateTask(Drivers::Hrs3300& heartRateSensor, Controllers::HeartRateController& controller);
      void Start();
      void Work();
      void PushMessage(Messages msg);

      void Register(System::SystemTask* systemTask, Controllers::DateTime* dateTimeController, Drivers::Bma421* motionSensor, Controllers::FS* fs);

    private:
      static void Process(void* instance);
      void StartMeasurement();
      void StopMeasurement();

      TaskHandle_t taskHandle;
      QueueHandle_t messageQueue;
      States state = States::Running;
      Drivers::Hrs3300& heartRateSensor;
      Controllers::HeartRateController& controller;
      Controllers::Ppg ppg;
      bool measurementStarted = false;

      Controllers::DateTime* dateTimeController = nullptr;
      System::SystemTask* systemTask = nullptr;
      Drivers::Bma421* motionSensor = nullptr;
      Controllers::FS* fs = nullptr;

      PPG_Data ppg_data_array[MAX_PPG_DATA_SIZE];
      size_t ppg_data_index = 0;
      size_t ppg_data_size = 0;

      void PushPPG_Data(PPG_Data data);
      void ClearPPG_Data();
      void SavePPG_Data();
    };

  }
}
