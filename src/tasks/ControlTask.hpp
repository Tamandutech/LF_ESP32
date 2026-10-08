#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "data_types.hpp"
#include "tasks/StateMachineTask.hpp"

class MotorDriver;
class VacuumDriver;
class IRSensorDriver;
class EncoderDriver;
class PathController;
class LedRgbDriver;
class ImuDriver;

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
  void calibrateGyroBias();
  void updatePose();
  void updateMarks();
  void appendMapPoint();
  void maybeRecordMapPoint();
  void recordTransitionLed();

  int32_t  encoderAverage() const;
  MapPoint currentMapPoint() const;

  StateMachineTask *stateMachine_;
  TaskHandle_t      taskHandle_;

  MotorDriver    *motorDriver_;
  VacuumDriver   *vacuumDriver_;
  IRSensorDriver *irSensorDriver_;
  EncoderDriver  *encoderLeft_;
  EncoderDriver  *encoderRight_;
  PathController *pathController_;
  LedRgbDriver   *ledRgbDriver_;
  ImuDriver      *imuDriver_;

  uint16_t lineSensorValues_[12];
  uint16_t sideSensorValues_[4];

  RobotState lastState_;
  uint32_t   mapPointIndex_;
  int32_t    finishLinePulses_;
  bool       properlyCalibrated_;
  bool       alternateLedColorFlag_;
  TickType_t lastIdleLedUpdate_;
  TickType_t lastMapSaveTick_;
  int64_t    maxCycleWorkUs_;

  // Odometria (modelo cinemático): pose, últimas velocidades e referências do
  // ciclo anterior.
  float   poseX_;
  float   poseY_;
  float   poseTheta_;
  float   lastV_;
  float   lastOmega_;
  float   gyroBias_;
  int32_t lastEncoderLeft_;
  int32_t lastEncoderRight_;
  int64_t lastPoseUpdateUs_;
  int64_t mappingStartUs_;

  // Marcações por borda de descida. Índice 0 = esquerda, 1 = direita.
  bool     markActive_[2];
  bool     markOtherSideSeen_[2];
  int64_t  markPulseStartUs_[2];
  uint16_t sector_;
  uint8_t  rightMarks_;
};
