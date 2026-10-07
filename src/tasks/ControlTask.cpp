#include "ControlTask.hpp"

#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include <cmath>

#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/ImuDriver/ImuDriver.hpp"
#include "drivers/LedRgbDriver/LedRgbDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"
#include "env.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "tasks/BluetoothTask.hpp"
#include "tasks/controllers/PathController.hpp"

namespace {
const char *TAG = "ControlTask";

constexpr uint8_t    kLineSensorCount    = 12;
constexpr uint8_t    kSideSensorCount    = 4;
constexpr TickType_t kIdleLedPeriodTicks = pdMS_TO_TICKS(250);

const uint8_t kMuxDigitalAddress[]  = GPIO_MULTIPLEXER_DIGITAL_ADDRESS;
const uint8_t kLineSensorMuxIndex[] = GPIO_MULTIPLEXER_LINE_SENSORS_INDEX;
const uint8_t kSideSensorMuxIndex[] = GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX;

const uint8_t kSideLeftPositions[]  = SIDE_SENSORS_LEFT_POSITIONS;
const uint8_t kSideRightPositions[] = SIDE_SENSORS_RIGHT_POSITIONS;
constexpr int kLeft                 = 0;
constexpr int kRight                = 1;

// Distância percorrida pela roda a cada pulso: 2πr / N_res, em mm.
constexpr float kMmPerPulse = 2.0F * 3.14159265F *
                              static_cast<float>(WHEEL_RADIUS) /
                              static_cast<float>(ENCODER_PULSES_PER_ROTATION);

/// true se algum sensor do lado está abaixo do limiar (vendo branco).
template <size_t N>
bool sideSeesMark(const uint16_t *values, const uint8_t (&positions)[N]) {
  for(size_t i = 0; i < N; i++) {
    if(values[positions[i]] < MARK_SENSOR_THRESHOLD) {
      return true;
    }
  }
  return false;
}
} // namespace

ControlTask::ControlTask(StateMachineTask *stateMachine)
    : stateMachine_(stateMachine), taskHandle_(nullptr), motorDriver_(nullptr),
      vacuumDriver_(nullptr), irSensorDriver_(nullptr), encoderLeft_(nullptr),
      encoderRight_(nullptr), pathController_(nullptr), ledRgbDriver_(nullptr),
      imuDriver_(nullptr), lineSensorValues_{}, sideSensorValues_{},
      lastState_(RobotState::IDLE), mapPointIndex_(0), finishLinePulses_(0),
      properlyCalibrated_(false), alternateLedColorFlag_(false),
      lastIdleLedUpdate_(0), lastMapSaveTick_(0), maxCycleWorkUs_(0),
      poseX_(0.0F), poseY_(0.0F), poseTheta_(0.0F), lastV_(0.0F),
      lastOmega_(0.0F), gyroBias_(0.0F), lastEncoderLeft_(0),
      lastEncoderRight_(0), lastPoseUpdateUs_(0), mappingStartUs_(0),
      markActive_{false, false}, markOtherSideSeen_{false, false},
      markPulseStartUs_{0, 0}, sector_(0), rightMarks_(0) {}

ControlTask::~ControlTask() {
  if(taskHandle_ != nullptr) {
    vTaskDelete(taskHandle_);
    taskHandle_ = nullptr;
  }
  delete imuDriver_;
  delete pathController_;
  delete ledRgbDriver_;
  delete irSensorDriver_;
  delete encoderRight_;
  delete encoderLeft_;
  delete vacuumDriver_;
  delete motorDriver_;
}

bool ControlTask::start(uint32_t stackSizeWords, UBaseType_t priority,
                        BaseType_t coreId) {
  if(taskHandle_ != nullptr || stateMachine_ == nullptr) {
    return false;
  }

  return xTaskCreatePinnedToCore(&ControlTask::taskEntry, "control_task",
                                 stackSizeWords, this, priority, &taskHandle_,
                                 coreId) == pdPASS;
}

void ControlTask::taskEntry(void *param) {
  auto *self = static_cast<ControlTask *>(param);
  self->run();
}

void ControlTask::initHardware() {
  // Defaults when NV storage has no PID/mapping values yet.
  if(globalData.parametersConfig.pidKp == 0.0F &&
     globalData.parametersConfig.pidKd == 0.0F) {
    globalData.parametersConfig.pidKp = 0.017F;
    globalData.parametersConfig.pidKd = 0.068F;
  }
  if(globalData.parametersConfig.mappingMotorPWM == 0) {
    globalData.parametersConfig.mappingMotorPWM = MOTOR_MAPPING_PWM;
  }
  if(globalData.parametersConfig.mapPointSaveInterval == 0) {
    globalData.parametersConfig.mapPointSaveInterval = MAP_POINT_SAVE_INTERVAL;
  }
  if(globalData.parametersConfig.mapPointMovingAverageSize == 0) {
    globalData.parametersConfig.mapPointMovingAverageSize =
        MAP_POINT_MOVING_AVERAGE_SIZE;
  }
  if(globalData.parametersConfig.mapPointDerivativeMargin == 0.0F) {
    globalData.parametersConfig.mapPointDerivativeMargin =
        MAP_POINT_DERIVATIVE_MARGIN;
  }

  const MotorPins motorPins = {.gpioDirectionA = GPIO_DIRECTION_A,
                               .gpioDirectionB = GPIO_DIRECTION_B,
                               .gpioPWMA       = GPIO_PWM_A,
                               .gpioPWMB       = GPIO_PWM_B};
  motorDriver_              = new MotorDriver(motorPins);

  const VacuumPins vacuumPins = {.gpioPWM = GPIO_PWM_VACUUM};
  vacuumDriver_               = new VacuumDriver(vacuumPins);

  const IRSensorParamSchema irParam = {
      .pins =
          {
                 .gpioMultiplexerDigitalAddress = kMuxDigitalAddress,
                 .gpioMultiplexerAnalogInput    = GPIO_MULTIPLEXER_ANALOG_INPUT,
                 },
      .lineSensorsCount            = kLineSensorCount,
      .lineSensorsMultiplexerIndex = kLineSensorMuxIndex,
      .sideSensorsCount            = kSideSensorCount,
      .sideSensorsMultiplexerIndex = kSideSensorMuxIndex,
      .multiplexerPinCount         = 4,
  };
  irSensorDriver_ = new IRSensorDriver(irParam);

  encoderLeft_ = new EncoderDriver(true);
  encoderLeft_->attachFullQuad(GPIO_ENCODER_LEFT_A, GPIO_ENCODER_LEFT_B);
  // Right wheel counts decrease when driving forward; invert so both increase.
  encoderRight_ = new EncoderDriver(true);
  encoderRight_->attachFullQuad(GPIO_ENCODER_RIGHT_A, GPIO_ENCODER_RIGHT_B);

  const LedRgbPins ledPins = {.gpioData =
                                  static_cast<gpio_num_t>(GPIO_LED_DEBUG),
                              .numLeds = NUM_LEDS_DEBUG};
  ledRgbDriver_            = new LedRgbDriver(ledPins);

  const ImuPins imuPins = {.gpioSda     = GPIO_IMU_SDA,
                           .gpioScl     = GPIO_IMU_SCL,
                           .frequencyHz = IMU_I2C_FREQ_HZ};
  imuDriver_            = new ImuDriver(imuPins);
  if(!imuDriver_->isReady()) {
    (void)bluetoothPushMessage(
        "Error: IMU nao encontrada (conferir GPIO_IMU_SDA/SCL no env.hpp). "
        "Omega sera gravado como 0.");
  }

  rebuildPathController();
}

void ControlTask::calibrateGyroBias() {
  gyroBias_ = 0.0F;
  if(imuDriver_ == nullptr || !imuDriver_->isReady()) {
    return;
  }

  // Fase 1 da calibração: robô parado. LED 0 em ciano.
  if(ledRgbDriver_ != nullptr) {
    ledRgbDriver_->setColor(0, LED_COLOR_CYAN);
    ledRgbDriver_->refresh();
  }
  (void)bluetoothPushMessage("Calibrando giroscopio: mantenha o robo parado");

  float sum   = 0.0F;
  int   count = 0;
  for(int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    float reading = 0.0F;
    if(imuDriver_->readGyroZ(&reading)) {
      sum += reading;
      count++;
    }
    vTaskDelay(pdMS_TO_TICKS(GYRO_BIAS_SAMPLE_INTERVAL_MS));
  }

  if(count > 0) {
    gyroBias_ = sum / static_cast<float>(count);
  }
  ESP_LOGI(TAG, "Bias do giroscopio: %.5f rad/s (%d leituras)",
           static_cast<double>(gyroBias_), count);
  (void)bluetoothPushMessage("Bias do giroscopio: %.5f rad/s (%d leituras)",
                             static_cast<double>(gyroBias_), count);
}

void ControlTask::calibrateSensors() {
  calibrateGyroBias();

  // Fase 2 da calibração: passar o robô sobre a linha. LED 0 em amarelo.
  (void)bluetoothPushMessage(
      "Calibrando sensores IR: passe o robo sobre a linha");
  if(ledRgbDriver_ != nullptr) {
    ledRgbDriver_->setColor(0, LED_COLOR_YELLOW);
    ledRgbDriver_->refresh();
  }

  ESP_LOGI(TAG, "Calibrando os sensores...");
  for(int i = 0; i < 50; i++) {
    irSensorDriver_->calibrate();
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  QTRSensors::CalibrationData *calibrationData =
      &irSensorDriver_->qtrSensors().calibrationOn;
  const uint8_t sensorCount = irSensorDriver_->getSensorCount();

  if(globalData.parametersConfig.hardcodedCalibration &&
     calibrationData->initialized && calibrationData->minimum != nullptr &&
     calibrationData->maximum != nullptr) {
    constexpr uint16_t kHardcodedIrMin = 200;
    constexpr uint16_t kHardcodedIrMax = 3600;
    for(uint8_t i = 0; i < sensorCount; i++) {
      calibrationData->minimum[i] = kHardcodedIrMin;
      calibrationData->maximum[i] = kHardcodedIrMax;
    }
  }

  bool calibrationOk = calibrationData->initialized &&
                       calibrationData->minimum != nullptr &&
                       calibrationData->maximum != nullptr;

  if(calibrationOk) {
    for(uint8_t i = 0; i < sensorCount; i++) {
      const uint16_t lo   = calibrationData->minimum[i];
      const uint16_t hi   = calibrationData->maximum[i];
      const int      diff = static_cast<int>(hi) - static_cast<int>(lo);
      ESP_LOGI(TAG,
               "Calibration sensor %u: min=%u max=%u diff=%d initialized=%d",
               static_cast<unsigned>(i), static_cast<unsigned>(lo),
               static_cast<unsigned>(hi), diff, calibrationData->initialized);
      if(diff < 50) {
        calibrationOk = false;
        (void)bluetoothPushMessage(
            "Error: Sensor %u calibrado incorretamente (range: %u, "
            "%u, difference: %d, initialized: %d)",
            static_cast<unsigned>(i), static_cast<unsigned>(lo),
            static_cast<unsigned>(hi), diff, calibrationData->initialized);
      }
    }
  }

  properlyCalibrated_ = calibrationOk;

  if(!calibrationOk) {
    ESP_LOGW(TAG, "Sensores nao calibrados corretamente");
    if(!calibrationData->initialized || calibrationData->minimum == nullptr ||
       calibrationData->maximum == nullptr) {
      (void)bluetoothPushMessage("Error: Sensores calibrados incorretamente "
                                 "(initialized: %d, pointers ok: %d)",
                                 calibrationData->initialized,
                                 calibrationData->minimum != nullptr &&
                                     calibrationData->maximum != nullptr);
    }
  } else {
    ESP_LOGI(TAG, "Sensores calibrados");
    (void)bluetoothPushMessage(
        "Sensores calibrados corretamente (%u sensores verificados, "
        "initialized: %d)",
        static_cast<unsigned>(sensorCount), calibrationData->initialized);
  }
}

void ControlTask::rebuildPathController() {
  delete pathController_;
  pathController_ = nullptr;

  const PathControllerConstants pidConstants = {
      .kP = globalData.parametersConfig.pidKp,
      .kI = globalData.parametersConfig.pidKi,
      .kD = globalData.parametersConfig.pidKd,
  };
  PathControllerParamSchema pathParam = {
      .constants      = pidConstants,
      .sensorQuantity = kLineSensorCount,
      .sensorValues   = lineSensorValues_,
      .maxAngle       = 45.0F,
      .radiusSensor   = 100,
      .sensorToCenter = 50,
  };
  pathController_ = new PathController(pathParam);
}

void ControlTask::stopActuators() {
  if(motorDriver_ != nullptr) {
    motorDriver_->pwmOutput(0, 0);
  }
  if(vacuumDriver_ != nullptr) {
    vacuumDriver_->pwmOutput(0);
  }
}

void ControlTask::onEnterMotionState(RobotState newState) {
  rebuildPathController();
  mapPointIndex_         = 0;
  finishLinePulses_      = 0;
  alternateLedColorFlag_ = false;
  maxCycleWorkUs_        = 0;
  if(newState == RobotState::MAPPING) {
    globalData.mapData.clear();
    globalData.mapData.reserve(256);
    lastMapSaveTick_ = xTaskGetTickCount();
  } else if(newState == RobotState::RUNNING && !globalData.mapData.empty()) {
    finishLinePulses_ = mapPointProgress(globalData.mapData.back());
  }
  if(encoderLeft_ != nullptr) {
    encoderLeft_->clearCount();
  }
  if(encoderRight_ != nullptr) {
    encoderRight_->clearCount();
  }
  if(newState == RobotState::MAPPING) {
    // Pose, contadores e detecção de bordas começam do zero a cada volta.
    poseX_            = 0.0F;
    poseY_            = 0.0F;
    poseTheta_        = 0.0F;
    lastV_            = 0.0F;
    lastOmega_        = 0.0F;
    lastEncoderLeft_  = 0;
    lastEncoderRight_ = 0;
    mappingStartUs_   = esp_timer_get_time();
    lastPoseUpdateUs_ = mappingStartUs_;
    sector_           = 0;
    rightMarks_       = 0;
    for(int side = kLeft; side <= kRight; side++) {
      markActive_[side]        = false;
      markOtherSideSeen_[side] = false;
      markPulseStartUs_[side]  = 0;
    }
  }
  if(ledRgbDriver_ != nullptr) {
    // RUNNING/MAPPING: LED 0 verde ao entrar (feedback de movimento).
    ledRgbDriver_->setColor(0, LED_COLOR_GREEN);
    ledRgbDriver_->refresh();
  }
}

void ControlTask::onLeaveMotionState(RobotState previousState) {
  if(previousState == RobotState::MAPPING) {
    appendMapPoint();
    (void)bluetoothPushMessage(
        "Mapeamento: %u pontos, %u setores, %u marcacoes direitas",
        static_cast<unsigned>(globalData.mapData.size()),
        static_cast<unsigned>(sector_), static_cast<unsigned>(rightMarks_));
  }
  (void)bluetoothPushMessage("Ciclo max: %lld us (periodo T: %d ms)",
                             static_cast<long long>(maxCycleWorkUs_),
                             CONTROL_LOOP_PERIOD_MS);
  stopActuators();
  lastIdleLedUpdate_ = 0;
}

void ControlTask::updateIdleLeds() {
  if(ledRgbDriver_ == nullptr) {
    return;
  }

  const TickType_t now = xTaskGetTickCount();
  if((now - lastIdleLedUpdate_) < kIdleLedPeriodTicks) {
    return;
  }
  lastIdleLedUpdate_ = now;

  if(properlyCalibrated_) {
    if(alternateLedColorFlag_) {
      ledRgbDriver_->setColor(0, LED_COLOR_PURPLE, 0.5f);
      alternateLedColorFlag_ = false;
    } else {
      ledRgbDriver_->setColor(0, LED_COLOR_WHITE, 0.5f);
      alternateLedColorFlag_ = true;
    }
    ledRgbDriver_->setColor(1, LED_COLOR_BLUE);
    ledRgbDriver_->setColor(2, LED_COLOR_BLUE);
    ledRgbDriver_->setColor(3, LED_COLOR_BLUE);
  } else {
    ledRgbDriver_->setColor(0, LED_COLOR_RED);
    ledRgbDriver_->setColor(1, LED_COLOR_RED);
    ledRgbDriver_->setColor(2, LED_COLOR_RED);
    ledRgbDriver_->setColor(3, LED_COLOR_RED);
  }
  ledRgbDriver_->refresh();
}

int32_t ControlTask::encoderAverage() const {
  if(encoderLeft_ == nullptr || encoderRight_ == nullptr) {
    return 0;
  }
  return (encoderLeft_->getCount() + encoderRight_->getCount()) / 2;
}

MapPoint ControlTask::currentMapPoint() const {
  MapPoint point;
  point.t =
      static_cast<uint32_t>((esp_timer_get_time() - mappingStartUs_) / 1000);
  point.encoderLeft  = encoderLeft_ != nullptr ? encoderLeft_->getCount() : 0;
  point.encoderRight = encoderRight_ != nullptr ? encoderRight_->getCount() : 0;
  point.v            = lastV_;
  point.omega        = lastOmega_;
  point.x            = poseX_;
  point.y            = poseY_;
  point.theta        = poseTheta_;
  point.sector       = sector_;
  point.rightMarks   = rightMarks_;
  point.speed = static_cast<float>(globalData.parametersConfig.mappingMotorPWM);
  return point;
}

void ControlTask::updatePose() {
  const int64_t now = esp_timer_get_time();
  const float   dt  = static_cast<float>(now - lastPoseUpdateUs_) * 1e-6F;
  lastPoseUpdateUs_ = now;
  if(dt <= 0.0F) {
    return;
  }

  // Pulsos de cada roda no intervalo (N_e, N_d).
  const int32_t left        = encoderLeft_->getCount();
  const int32_t right       = encoderRight_->getCount();
  const int32_t pulsesLeft  = left - lastEncoderLeft_;
  const int32_t pulsesRight = right - lastEncoderRight_;
  lastEncoderLeft_          = left;
  lastEncoderRight_         = right;

  // v_e = 2πr N_e / (N_res Δt), v_d = 2πr N_d / (N_res Δt), v = (v_d + v_e)/2.
  const float vLeft  = kMmPerPulse * static_cast<float>(pulsesLeft) / dt;
  const float vRight = kMmPerPulse * static_cast<float>(pulsesRight) / dt;
  const float v      = (vRight + vLeft) / 2.0F;

  // ω = ω_z − b, pelo giroscópio. Se a leitura falhar, repete o último ω.
  float omega = lastOmega_;
  float gyroZ = 0.0F;
  if(imuDriver_ != nullptr && imuDriver_->readGyroZ(&gyroZ)) {
    omega = IMU_GYRO_Z_SIGN * (gyroZ - gyroBias_);
  } else if(imuDriver_ == nullptr || !imuDriver_->isReady()) {
    omega = 0.0F;
  }

  // Modelo de estimativa discretizado.
  poseX_ += v * dt * cosf(poseTheta_);
  poseY_ += v * dt * sinf(poseTheta_);
  poseTheta_ += omega * dt;

  lastV_     = v;
  lastOmega_ = omega;
}

void ControlTask::updateMarks() {
  const int64_t now       = esp_timer_get_time();
  const bool    active[2] = {
      sideSeesMark(sideSensorValues_, kSideLeftPositions),
      sideSeesMark(sideSensorValues_, kSideRightPositions),
  };

  for(int side = kLeft; side <= kRight; side++) {
    const int other = side == kLeft ? kRight : kLeft;
    if(active[side]) {
      if(!markActive_[side]) {
        // Início do pulso deste lado.
        markActive_[side]        = true;
        markOtherSideSeen_[side] = false;
        markPulseStartUs_[side]  = now;
      }
      if(active[other]) {
        markOtherSideSeen_[side] = true;
      }
    } else if(markActive_[side]) {
      // Borda de descida: o pulso deste lado terminou.
      markActive_[side] = false;
      const bool longEnough =
          (now - markPulseStartUs_[side]) >= MARK_MIN_PULSE_US;
      // Se o outro lado ficou ativo durante o pulso, é cruzamento: ignora.
      if(longEnough && !markOtherSideSeen_[side]) {
        if(side == kLeft) {
          sector_++;
        } else {
          rightMarks_++;
        }
      }
      markOtherSideSeen_[side] = false;
    }
  }
}

void ControlTask::recordTransitionLed() {
  alternateLedColorFlag_ = !alternateLedColorFlag_;
  if(ledRgbDriver_ != nullptr) {
    ledRgbDriver_->setColor(0, alternateLedColorFlag_ ? LED_COLOR_ORANGE
                                                      : LED_COLOR_CYAN);
    ledRgbDriver_->refresh();
  }
}

void ControlTask::appendMapPoint() {
  if(globalData.mapData.size() >= static_cast<size_t>(MAP_POINT_MAX_COUNT)) {
    return;
  }
  globalData.mapData.push_back(currentMapPoint());
  lastMapSaveTick_ = xTaskGetTickCount();
}

void ControlTask::maybeRecordMapPoint() {
  const TickType_t now = xTaskGetTickCount();
  int32_t intervalMs   = globalData.parametersConfig.mapPointSaveInterval;
  if(intervalMs < 1) {
    intervalMs = 1;
  }
  if((now - lastMapSaveTick_) >=
     pdMS_TO_TICKS(static_cast<uint32_t>(intervalMs))) {
    appendMapPoint();
    recordTransitionLed();
  }
}

void ControlTask::tickStopped() {
  stopActuators();
  updateIdleLeds();
}

void ControlTask::tickRunning() {
  const int32_t progress = encoderAverage();

  if(finishLinePulses_ > 0 && progress > finishLinePulses_) {
    stopActuators();
    if(ledRgbDriver_ != nullptr) {
      ledRgbDriver_->setColor(0, LED_COLOR_RED);
      ledRgbDriver_->refresh();
    }
    if(stateMachine_ != nullptr) {
      const Event stopEvent{EventType::STOP};
      (void)stateMachine_->postEvent(stopEvent, 0);
    }
    return;
  }

  irSensorDriver_->readCalibrated(lineSensorValues_, sideSensorValues_);

  if(!globalData.mapData.empty() &&
     progress > mapPointProgress(globalData.mapData[mapPointIndex_]) &&
     (mapPointIndex_ + 1U) < globalData.mapData.size()) {
    mapPointIndex_++;
    alternateLedColorFlag_ = !alternateLedColorFlag_;
    if(ledRgbDriver_ != nullptr) {
      ledRgbDriver_->setColor(0, alternateLedColorFlag_ ? LED_COLOR_ORANGE
                                                        : LED_COLOR_CYAN);
      ledRgbDriver_->refresh();
    }
  }

  const float   pathPid = pathController_->getPID();
  const int32_t basePwm =
      globalData.mapData.empty()
          ? 0
          : static_cast<int32_t>(globalData.mapData[mapPointIndex_].speed);

  motorDriver_->pwmOutput(basePwm + static_cast<int32_t>(pathPid),
                          basePwm - static_cast<int32_t>(pathPid));
  vacuumDriver_->pwmOutput(globalData.parametersConfig.vacuumPWM);
}

void ControlTask::tickMapping() {
  // Chegada: a segunda marcação à direita encerra o mapeamento.
  if(rightMarks_ >= 2) {
    stopActuators();
    if(ledRgbDriver_ != nullptr) {
      ledRgbDriver_->setColor(0, LED_COLOR_RED);
      ledRgbDriver_->refresh();
    }
    if(stateMachine_ != nullptr) {
      const Event stopEvent{EventType::STOP};
      (void)stateMachine_->postEvent(stopEvent, 0);
    }
    return;
  }

  irSensorDriver_->readCalibrated(lineSensorValues_, sideSensorValues_);

  const float pathPid = pathController_->getPID();
  const float base =
      static_cast<float>(globalData.parametersConfig.mappingMotorPWM);
  motorDriver_->pwmOutput(static_cast<int32_t>(base + pathPid),
                          static_cast<int32_t>(base - pathPid));
  vacuumDriver_->pwmOutput(globalData.parametersConfig.vacuumPWM);

  updatePose();
  updateMarks();
  maybeRecordMapPoint();
}

void ControlTask::run() {
  initHardware();
  stopActuators();
  calibrateSensors();
  lastIdleLedUpdate_ = 0;
  ESP_LOGI(TAG, "control loop ready");

  // Período fixo T do laço (vTaskDelayUntil). O Δt da odometria continua
  // sendo medido a cada ciclo em updatePose().
  TickType_t       lastWake = xTaskGetTickCount();
  const TickType_t period   = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS) > 0
                                ? pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS)
                                : 1;

  for(;;) {
    const int64_t    cycleStartUs = esp_timer_get_time();
    const RobotState state        = gRobotState;

    if(state != lastState_) {
      const bool wasMoving = lastState_ == RobotState::RUNNING ||
                             lastState_ == RobotState::MAPPING;
      const bool isMoving =
          state == RobotState::RUNNING || state == RobotState::MAPPING;
      if(isMoving && !wasMoving) {
        onEnterMotionState(state);
      } else if(!isMoving && wasMoving) {
        onLeaveMotionState(lastState_);
      } else if(isMoving && wasMoving) {
        onLeaveMotionState(lastState_);
        onEnterMotionState(state);
      }
      lastState_ = state;
    }

    switch(state) {
    case RobotState::RUNNING: tickRunning(); break;
    case RobotState::MAPPING: tickMapping(); break;
    case RobotState::IDLE:
    case RobotState::CALIBRATING:
      // Calibração de sensores já ocorreu em calibrateSensors().
      tickStopped();
      break;
    }

    // Duração do trabalho do ciclo, para escolher T (informada ao fim da
    // volta). Sem printf / fila bloqueante no loop crítico.
    const int64_t workUs = esp_timer_get_time() - cycleStartUs;
    if(workUs > maxCycleWorkUs_) {
      maxCycleWorkUs_ = workUs;
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
