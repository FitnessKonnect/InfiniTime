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

  // Open the root directory
  if (fs.DirOpen("/", &dir) == LFS_ERR_OK) {
    SEGGER_RTT_printf(0, "listing files\r\n-------\r\n");
    // Iterate through the entries in the directory
    while (fs.DirRead(&dir, &info) != 0) {
      if (info.type == LFS_TYPE_REG) { // Regular file
        SEGGER_RTT_printf(0, "/%s\r\n", info.name);
      } else if (info.type == LFS_TYPE_DIR) { // Directory
        SEGGER_RTT_printf(0, "%s/\r\n", info.name);
      }
    }

    // Close the directory
    fs.DirClose(&dir);
    SEGGER_RTT_printf(0, "-------\r\n");
  } else {
    SEGGER_RTT_printf(0, "Error opening root directory\r\n");
  }

  // Ensure the "/.rinzler" directory exists
  if (fs.DirOpen("/.rinzler", &dir) != LFS_ERR_OK) {
    // Directory doesn't exist, create it
    if (fs.DirCreate("/.rinzler") != LFS_ERR_OK) {
      SEGGER_RTT_printf(0, "Error creating /.rinzler directory\r\n");
      return;
    }
  } else {
    // Directory already exists, close it
    fs.DirClose(&dir);
  }

  // Try to create "/.rinzler/test" file
  lfs_file_t file;
  if (fs.FileOpen(&file, "/.rinzler/test", LFS_O_RDWR) != LFS_ERR_OK) {
    // File doesn't exist, create it
    if (fs.FileOpen(&file, "/.rinzler/test", LFS_O_RDWR | LFS_O_CREAT) == LFS_ERR_OK) {
      SEGGER_RTT_printf(0, "Created /.rinzler/test file\r\n");
      fs.FileClose(&file);
    } else {
      SEGGER_RTT_printf(0, "Error creating /.rinzler/test file\r\n");
    }
  } else {
    // File already exists, just close it
    SEGGER_RTT_printf(0, "/.rinzler/test File already exists, just close it\r\n");
    fs.FileClose(&file);
  }
}

#endif //__FKPPGTASK_CPP__
