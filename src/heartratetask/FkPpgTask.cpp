#ifndef __FKPPGTASK_CPP__
#define __FKPPGTASK_CPP__
#include "heartratetask/FkPpgTask.h"
#include <task.h>
#include "SEGGER_RTT.h"
#include <components/fs/FS.h>
#include <cctype>

using namespace Pinetime::Applications;

FkPpgTask::FkPpgTask(Drivers::Hrs3300& heartRateSensor, Pinetime::Drivers::Bma421& motionSensor)
  : heartRateSensor(heartRateSensor), motionSensor(motionSensor) {

  this->waitTimer = xTimerCreate("PPG-Wait", pdMS_TO_TICKS(CYCLE_DURATION), pdTRUE, this, FkPpgTask::StartMeasurementFK);
  this->measurementTimer =
    xTimerCreate("PPG-Measurement", pdMS_TO_TICKS(MEASUREMENT_DURATION), pdFALSE, this, FkPpgTask::StopMeasurementFK);

  if (this->waitTimer != NULL) {
    SEGGER_RTT_printf(0, "waitTimer was created successfully.\r\n");
  }

  if (!this->measurementTimer) {
    SEGGER_RTT_printf(0, "failed to create measurement timers\r\n");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

void FkPpgTask::StartFK(HeartRateTask& heartRateTaskRef, Pinetime::Controllers::FS& fs) {
  SEGGER_RTT_printf(0, "starting ppg task\r\n");
  this->listMatchingFiles(fs);
  this->heartRateTask = &heartRateTaskRef;
  this->EnableMeasurementFK();
}

void FkPpgTask::EnableMeasurementFK() {
  SEGGER_RTT_printf(0, "enabling measurement\r\n");
  this->measure = true;
  this->state = States::Running;

  BaseType_t startResult = xTimerStart(this->waitTimer, 0);
  if (startResult != pdPASS) {
    SEGGER_RTT_printf(0, "Failed to start waitTimer with error code: %d\r\n", startResult);
  }
}

void FkPpgTask::DisableMeasurementFK() {
  SEGGER_RTT_printf(0, "disabling measurement\r\n");
  this->heartRateSensor.Disable();
  this->measure = false;
  this->state = States::Stopped;
  xTimerStop(this->waitTimer, portMAX_DELAY);
  xTimerStop(this->measurementTimer, portMAX_DELAY);
}

void FkPpgTask::StartMeasurementFK(TimerHandle_t xTimer) {
  SEGGER_RTT_printf(0, "starting measurement timer\r\n");
  auto* instance = static_cast<FkPpgTask*>(pvTimerGetTimerID(xTimer));
  if (!instance->heartRateTask) {
    SEGGER_RTT_printf(0, "StartMeasurementFK: heartRateTask is null\r\n");
    xTimerStop(instance->waitTimer, 0);
    return;
  }

  instance->state = States::Measuring;
  instance->measure = true;
  BaseType_t startResult = xTimerStart(instance->measurementTimer, 0);
  if (startResult != pdPASS) {
    SEGGER_RTT_printf(0, "Failed to start measurementTimer with error code: %d\r\n", startResult);
  }

  vTaskDelay(100);
  SEGGER_RTT_printf(0, "starting measurement\r\n");
  instance->heartRateTask->PushMessage(Pinetime::Applications::HeartRateTask::Messages::StartMeasurement);
}

void FkPpgTask::StopMeasurementFK(TimerHandle_t xTimer) {
  SEGGER_RTT_printf(0, "stopping measurement\r\n");
  auto* instance = static_cast<FkPpgTask*>(pvTimerGetTimerID(xTimer));
  if (!instance->heartRateTask) {
    SEGGER_RTT_printf(0, "StopMeasurementFK: heartRateTask is null\r\n");
    return;
  }
  instance->measure = false;
  instance->state = States::Waiting;
  xTimerStop(instance->measurementTimer, 0);
  SEGGER_RTT_printf(0, "stopped measurement\r\n");
  instance->heartRateTask->PushMessage(Pinetime::Applications::HeartRateTask::Messages::StopMeasurement);
}

void FkPpgTask::listMatchingFiles(Pinetime::Controllers::FS& fs) {
  lfs_dir_t dir;
  lfs_info info;

  if (fs.DirOpen("/", &dir) != LFS_ERR_OK) {
    return; // Failed to open directory
  }

  while (fs.DirRead(&dir, &info) == LFS_ERR_OK) {
    // Check if the entry is a file (there's no direct type check in the provided API, so we'll assume non-directory entries are files)
    if (info.type != LFS_TYPE_DIR) {
      // Check if filename matches the pattern: fkppg_##_##_##.csv
      const char* name = info.name;
      if (name[0] == 'f' && name[1] == 'k' && name[2] == 'p' && name[3] == 'p' && name[4] == 'g' && name[5] == '_' && isdigit(name[6]) &&
          isdigit(name[7]) && name[8] == '_' && isdigit(name[9]) && isdigit(name[10]) && name[11] == '_' && isdigit(name[12]) &&
          isdigit(name[13]) && name[14] == '.' && name[15] == 'c' && name[16] == 's' && name[17] == 'v') {
        SEGGER_RTT_printf(0, "found matching file: %s\r\n", name);
      }
    }
  }

  fs.DirClose(&dir);
}

#endif //__FKPPGTASK_CPP__
