#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <stdint.h>

/// Ponto do mapa, gravado pelo pipeline de mapeamento com o modelo cinemático
/// do robô diferencial. O índice é a posição no array (RAM) / o campo `index`
/// no arquivo e na comunicação Bluetooth.
struct MapPoint {
  uint32_t t{};            ///< Tempo desde a largada, em ms.
  int32_t  encoderLeft{};  ///< Pulsos acumulados da roda esquerda (bruto).
  int32_t  encoderRight{}; ///< Pulsos acumulados da roda direita (bruto).
  float    v{};            ///< Velocidade linear pelos encoders, em mm/s.
  float    omega{};        ///< Velocidade angular pela IMU, sem bias, em rad/s.
  float    x{};            ///< Posição x, em mm.
  float    y{};            ///< Posição y, em mm.
  float    theta{};        ///< Orientação, em rad.
  uint16_t sector{};       ///< Marcações esquerdas válidas já vistas.
  uint8_t  rightMarks{};   ///< Marcações direitas válidas já vistas (0, 1, 2).
  float    speed{};        ///< PWM base do ponto (%).
};

/// Progresso para frente: média das contagens dos encoders (pulsos).
inline int32_t mapPointProgress(const MapPoint &point) {
  return (point.encoderLeft + point.encoderRight) / 2;
}

struct ParametersConfig {
  bool    runOnMappingMode{};
  int32_t vacuumPWM{};
  /// When true, line IR calibration min/max are forced to fixed values after
  /// \c calibrate() (see MainTask). Set via CLI \c
  /// Calibration.hardcodedCalibration.
  bool hardcodedCalibration{};
  /// Line-follow PID gains (PathController). Set via CLI \c PID.kP, \c PID.kI,
  /// \c PID.kD.
  float pidKp{};
  float pidKi{};
  float pidKd{};
  /// Base motor PWM magnitude while mapping (see MainTask MAPPING state).
  /// BLE: \c Mapping.mappingMotorPWM (clamped to \c MAX_MOTOR_PWM from
  /// env.hpp).
  int32_t mappingMotorPWM{};
  /// Minimum interval between recorded map points, in milliseconds.
  /// BLE: \c Mapping.mapPointSaveInterval.
  int32_t mapPointSaveInterval{};
  /// Sample window for the moving average of encoder-delta derivatives.
  /// BLE: \c Mapping.mapPointMovingAverageSize.
  int32_t mapPointMovingAverageSize{};
  /// Margin between current derivative and moving average to record a
  /// straight/curve transition. BLE: \c Mapping.mapPointDerivativeMargin.
  float mapPointDerivativeMargin{};
};

#endif // DATA_TYPES_HPP
