#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "data_types.hpp"
#include "tasks/StateMachineTask.hpp"

#include <DataTomeMvAvg.h>

class MotorDriver;
class VacuumDriver;
class IRSensorDriver;
class EncoderDriver;
class PathController;
class LedRgbDriver;

/**
 * Loop crítico de controle (Core 1): não é Active Object com fila.
 * Poll atômico de gRobotState — FSM só publica o estado; este loop decide
 * parar (IDLE/CALIBRATING) ou andar com PID (RUNNING/MAPPING).
 * Calibração IR roda uma vez na inicialização, fora do loop.
 */
class ControlTask {
public:
  explicit ControlTask(StateMachineTask *stateMachine);
  ~ControlTask();

  bool start(uint32_t stackSizeWords = 6144, UBaseType_t priority = 10,
             BaseType_t coreId = 1);

private:
  static void taskEntry(void *param);
  void        run();

  void initHardware();
  void calibrateSensors();
  void rebuildPathController();
  void stopActuators();
  void onEnterMotionState(RobotState newState);
  void onLeaveMotionState(RobotState previousState);
  void tickStopped();
  void tickRunning();
  void tickMapping();
  void updateIdleLeds();
  void appendMapPoint(MapPoint::PointType pointType, float encoderDerivative,
                      float encoderDerivativeAverage);
  void maybeRecordMapPoint();
  void resetMappingDerivative();
  void recordTransitionLed();

  int32_t  encoderAverage() const;
  MapPoint currentMapPoint(MapPoint::PointType pointType, float encoderDerivative,
                           float encoderDerivativeAverage) const;

  StateMachineTask *stateMachine_;
  TaskHandle_t      taskHandle_;

  MotorDriver    *motorDriver_;
  VacuumDriver   *vacuumDriver_;
  IRSensorDriver *irSensorDriver_;
  EncoderDriver  *encoderLeft_;
  EncoderDriver  *encoderRight_;
  PathController *pathController_;
  LedRgbDriver   *ledRgbDriver_;

  uint16_t lineSensorValues_[12];
  uint16_t sideSensorValues_[4];

  RobotState            lastState_;
  uint32_t              mapPointIndex_;
  int32_t               finishLinePulses_;
  bool                  properlyCalibrated_;
  bool                  alternateLedColorFlag_;
  TickType_t            lastIdleLedUpdate_;
  int32_t               lastMapSaveProgress_;
  int64_t               lastDerivativeTimeUs_;
  int32_t               lastDeltaEncoder_;
  float                 lastEncoderDerivative_;
  float                 lastEncoderDerivativeAverage_;
  bool                  derivativeInitialized_;
  DataTomeMvAvg<float> *encoderDerivativeAverage_;
};
