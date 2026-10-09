#pragma once

#include <cstdint>

#define ROBOT_WIDTH                 (4)
#define WHEEL_RADIUS                (11)
#define WHEEL_CIRCUMFERENCE         (70)
#define ENCODER_PULSES_PER_ROTATION (4095)

#define MOTOR_MAPPING_PWM (10)
#define VACUUM_BASE_PWM   (100)

#define MAX_MOTOR_PWM (66)

/// Minimum encoder-pulse distance (average of both wheels) between recorded
/// map points while mapping.
#define MAP_POINT_SAVE_PULSE_INTERVAL (16384) // 2 full rotations
/// Window size for the moving average of encoder-delta derivatives.
#define MAP_POINT_MOVING_AVERAGE_SIZE (4)
/// |current derivative - moving average| must exceed this to record a
/// transition.
#define MAP_POINT_DERIVATIVE_MARGIN   (5.0F)
/// Cap on RAM/flash map size (periodic samples + ponto de parada).
#define MAP_POINT_MAX_COUNT           (2048)

#define SIDE_SENSOR_READ_AVERAGE_COUNT (5)

#define EPSILON_TOLERANCE \
  (1e-6F) // Tolerância para comparações de ponto flutuante

// Constantes para prevenção de integral windup no PID
#define INTEGRAL_MAX (1000.0F)  // Valor máximo para o termo integral
#define INTEGRAL_MIN (-1000.0F) // Valor mínimo para o termo integral

#define GPIO_LED_DEBUG (47)
#define NUM_LEDS_DEBUG (4)

#define GPIO_BATTERY_VOLTAGE (18)

#define GPIO_DIRECTION_A (9)
#define GPIO_DIRECTION_B (37)
#define GPIO_PWM_A       (3)
#define GPIO_PWM_B       (38)

#define GPIO_PWM_VACUUM (11)

#define GPIO_ENCODER_LEFT_A  (7)
#define GPIO_ENCODER_LEFT_B  (6)
#define GPIO_ENCODER_RIGHT_A (12)
#define GPIO_ENCODER_RIGHT_B (13)

#define GPIO_MULTIPLEXER_DIGITAL_ADDRESS {39, 40, 41, 42}
#define GPIO_MULTIPLEXER_ANALOG_INPUT    (10)
#define GPIO_MULTIPLEXER_LINE_SENSORS_INDEX \
  {13, 12, 11, 10, 9, 8, 5, 4, 3, 2, 1, 0}
#define GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX {15, 14, 6, 7}
