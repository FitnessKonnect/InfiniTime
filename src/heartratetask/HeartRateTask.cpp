#include "heartratetask/HeartRateTask.h"
#include <drivers/Hrs3300.h>
#include <drivers/Bma421.h>
#include <components/heartrate/HeartRateController.h>
#include <components/datetime/DateTimeController.h>
#include <nrf_log.h>
#include <systemtask/SystemTask.h>
#include "SEGGER_RTT.h"
#include <littlefs/lfs.h>

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
      int16_t motion_x = 0;
      int16_t motion_y = 0;
      int16_t motion_z = 0;

      if (motionSensor != nullptr) {
        auto motion = motionSensor->Process();
        motion_x = motion.x;
        motion_y = motion.y;
        motion_z = motion.z;
      } else {
        SEGGER_RTT_printf(0, "motion sensor is null\r\n");
      }

      SEGGER_RTT_printf(0, "current hr, als, x, y, z: %u, %u, %d, %d, %d\r\n", hrs, als, motion_x, motion_y, motion_z);
      PushPPG_Data({hrs, als, motion_x, motion_y, motion_z});
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
  SavePPG_Data();
  ClearPPG_Data();
  vTaskDelay(100);
}

void HeartRateTask::Register(System::SystemTask* systemTask,
                             Controllers::DateTime* dateTimeController,
                             Drivers::Bma421* motionSensor,
                             Controllers::FS* fs) {
  this->systemTask = systemTask;
  this->dateTimeController = dateTimeController;
  this->motionSensor = motionSensor;
  this->fs = fs;
}

void HeartRateTask::PushPPG_Data(PPG_Data data) {
  if (ppg_data_index < MAX_PPG_DATA_SIZE) {
    ppg_data_array[ppg_data_index] = data;
    ppg_data_index++;
    ppg_data_size++;
    return;
  }
  SEGGER_RTT_printf(0, "PPG data array is full\r\n");
}

void HeartRateTask::ClearPPG_Data() {
  ppg_data_index = 0;
  ppg_data_size = 0;
  // clear array
  for (int i = 0; i < MAX_PPG_DATA_SIZE; i++) {
    ppg_data_array[i] = {0, 0, 0, 0, 0};
  }
  SEGGER_RTT_printf(0, "PPG data array cleared\r\n");
}

void HeartRateTask::SavePPG_Data() {
  if (ppg_data_size == 0) {
    SEGGER_RTT_printf(0, "PPG data array is empty\r\n");
    return;
  }
  if (dateTimeController == nullptr) {
    SEGGER_RTT_printf(0, "DateTimeController is null\r\n");
    return;
  }
  if (fs == nullptr) {
    SEGGER_RTT_printf(0, "FS is null\r\n");
    return;
  }

  // get current time

  // get an int with DDMM, and an int with HHMM
  int year = dateTimeController->Year();

  if (year < 2023) {
    SEGGER_RTT_printf(0, "Year is less than 2023 ... should wait for timeSync\r\n");
    //   FIXME: add the logic before going to production
    //     return;
  }

  Pinetime::Controllers::DateTime::Months _month = dateTimeController->Month();
  uint8_t month = static_cast<uint8_t>(_month);
  uint8_t day = dateTimeController->Day();
  uint8_t hour = dateTimeController->Hours();
  uint8_t minute = dateTimeController->Minutes();

  char filename[32];
  snprintf(filename, sizeof(filename), "/fkppg_%02d%02d_%02d.csv", month, day, hour);
  lfs_file_t file;

  // open file
  if (!fs->Stat(filename, nullptr)) { // Use the Stat method from FS
    // File doesn't exist, create a new one
    SEGGER_RTT_printf(0, "Creating new file for the current hour: %s\n", filename);
    if (!fs->FileOpen(&file, filename, LFS_O_RDWR)) {
      return;
    }

    // Write CSV header for the new file
    char header[] = "Timestamp, HRS, ALS, Motion X, Motion Y, Motion Z\n";
    fs->FileWrite(&file, reinterpret_cast<const uint8_t*>(header), sizeof(header) - 1);
  } else {
    // File exists, append data to it
    SEGGER_RTT_printf(0, "Appending to the existing file for the current hour: %s\n", filename);
    if (!fs->FileOpen(&file, filename, LFS_O_RDWR)) {
      return;
    }
  }

  auto info = fs->GetFSSize();
  SEGGER_RTT_printf(0, "Size(before dump) of the file %s: %d bytes\n", filename, info);

  // dump data
  for (size_t i = 0; i < ppg_data_index; i++) {
    char buffer[128]; // Adjust buffer size if needed
    int len = snprintf(buffer,
                       sizeof(buffer),
                       "%02d%02d%02d%02d,%lu,%lu,%d,%d,%d\n",
                       month,
                       day,
                       hour,
                       minute,
                       ppg_data_array[i].hrs,
                       ppg_data_array[i].als,
                       ppg_data_array[i].motion_x,
                       ppg_data_array[i].motion_y,
                       ppg_data_array[i].motion_z);
    fs->FileWrite(&file, reinterpret_cast<const uint8_t*>(buffer), len);
  }

  // Close the file
  fs->FileClose(&file); // Adjust the call

  // 5. Print the size of the file
  info = fs->GetFSSize();
  SEGGER_RTT_printf(0, "Size of the file %s: %d bytes\n", filename, info);
}
