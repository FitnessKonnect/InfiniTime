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

int PrintDir(const char* path, Pinetime::Controllers::FS& fs) {
  lfs_dir_t dir;
  lfs_info info;
  int fsOpRes = LFS_ERR_OK;

  if ((fsOpRes = fs.DirOpen(path, &dir)) == LFS_ERR_OK) {
    SEGGER_RTT_printf(0, "listing %s\r\n-------\r\n", path);
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
    SEGGER_RTT_printf(0, "ERR: List %s dir failed, fsOpRes: %d\r\n", path, fsOpRes);
    return fsOpRes;
  }
  return LFS_ERR_OK;
}

int PrintFileData(const char* path, Pinetime::Controllers::FS& fs) {
  lfs_file_t file;
  lfs_info info;
  char buffer[128]; // Buffer to read file data into
  int fsOpRes;

  // Open the file
  fsOpRes = fs.FileOpen(&file, path, LFS_O_RDONLY);
  if (fsOpRes != LFS_ERR_OK) {
    SEGGER_RTT_printf(0, "ERR: Failed to open file %s, fsOpRes: %d\r\n", path, fsOpRes);
    return fsOpRes;
  }

  // Get file information
  fsOpRes = fs.Stat(path, &info);
  if (fsOpRes != LFS_ERR_OK) {
    SEGGER_RTT_printf(0, "ERR: Failed to get file info for %s, fsOpRes: %d\r\n", path, fsOpRes);
    fs.FileClose(&file);
    return fsOpRes;
  }

  SEGGER_RTT_printf(0, "Reading file %s, size: %d bytes\r\n", path, info.size);

  // Print the starting line for the file content
  SEGGER_RTT_printf(0, "File %s ------------\r\n", path);

  // Read and print the file data
  while (true) {
    int bytesRead = fs.FileRead(&file, reinterpret_cast<uint8_t*>(buffer), sizeof(buffer) - 1);
    if (bytesRead < 0) {
      SEGGER_RTT_printf(0, "ERR: Failed to read file %s, fsOpRes: %d\r\n", path, bytesRead);
      fsOpRes = bytesRead;
      break;
    }

    if (bytesRead == 0) {
      // End of file
      break;
    }

    // Null-terminate the read data to safely print it
    buffer[bytesRead] = '\0';
    SEGGER_RTT_printf(0, "%s", buffer);
  }

  // Print the ending line for the file content
  SEGGER_RTT_printf(0, "------------\r\n");

  // Close the file
  fs.FileClose(&file);

  return fsOpRes;
}

int DeleteFile(const char* path, Pinetime::Controllers::FS& fs) {
  int fsOpRes = LFS_ERR_OK;

  // Delete the file
  fsOpRes = fs.FileDelete(path);
  if (fsOpRes != LFS_ERR_OK) {
    SEGGER_RTT_printf(0, "ERR: Failed to delete file %s, fsOpRes: %d\r\n", path, fsOpRes);
  }
  return fsOpRes;
}

void FkPpgTask::listMatchingFiles(Pinetime::Controllers::FS& fs) {
  int fsOpRes = LFS_ERR_OK;

  if ((fsOpRes = PrintDir("/fk", fs)) == LFS_ERR_OK) {
    // FIXME: remove this before going to prod
    PrintFileData("/fk/fkppg_0101_00.csv", fs);
    DeleteFile("/fk/fkppg_0101_00.csv", fs);
  } else {
    fsOpRes = fs.DirCreate("/fk");
    if (fsOpRes != LFS_ERR_OK) {
      SEGGER_RTT_printf(0, "Error creating FK directory, fsOpRes: %d\r\n", fsOpRes);
    } else {
      SEGGER_RTT_printf(0, "FK directory created\r\n");
    }
  }
}

#endif //__FKPPGTASK_CPP__
