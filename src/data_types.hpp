#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <stdint.h>

/// Ponto do mapa. O índice é a posição no array (RAM) / o campo `index` no
/// arquivo e na comunicação Bluetooth.
/// `encoderLeft` / `encoderRight` são as contagens das rodas; `speed` é o PWM
/// base do motor (equivalente ao antigo `baseMotorPWM`).
/// `encoderDerivative` é a derivada de (right - left) no instante do ponto;
/// `encoderDerivativeAverage` é a média móvel dessa derivada.
struct MapPoint {
  int32_t encoderLeft{};
  int32_t encoderRight{};
  float   encoderDerivative{};
  float   encoderDerivativeAverage{};
  float   speed{};
  enum PointType {
    AUTO_MARK,         ///< Recorded by the robot while mapping.
    MANUAL_MARK,       ///< Added by the user via CLI.
    STOP_COMMAND_MARK, ///< Last point, saved when mapping is stopped.
    UNKNOWN_MARK,      ///< Unknown mark type.
    CURVE_START_MARK,  ///< Transition into a curve (derivative above average).
    CURVE_END_MARK ///< Transition out of a curve (derivative below average).
  } pointType{UNKNOWN_MARK};
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
  /// Minimum distance between recorded map points, in encoder pulses
  /// (average of both wheels). BLE: \c Mapping.mapPointSavePulseInterval.
  int32_t mapPointSavePulseInterval{};
  /// Sample window for the moving average of encoder-delta derivatives.
  /// BLE: \c Mapping.mapPointMovingAverageSize.
  int32_t mapPointMovingAverageSize{};
  /// Margin between current derivative and moving average to record a
  /// straight/curve transition. BLE: \c Mapping.mapPointDerivativeMargin.
  float mapPointDerivativeMargin{};
};

#endif // DATA_TYPES_HPP
