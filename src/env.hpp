#pragma once

#include <cstdint>

// Parâmetros do modelo cinemático (ver documento do pipeline de mapeamento).
// A CONFERIR no robô antes do teste:
// - WHEEL_RADIUS: raio das rodas (r_d = r_e), em mm.
// - ENCODER_PULSES_PER_ROTATION: pulsos por volta da RODA (N_res), já
//   considerando a redução e a contagem em quadratura.
// - ROBOT_WIDTH: distância entre as rodas (L). Não entra no modelo, porque ω
//   vem da IMU. O valor 4 não corresponde a milímetros.
#define ROBOT_WIDTH                 (4)
#define WHEEL_RADIUS                (11)
#define WHEEL_CIRCUMFERENCE         (70)
#define ENCODER_PULSES_PER_ROTATION (4095)

// Período fixo T do laço de controle, em ms (granularidade do tick: 1 ms).
// Ajustar pelo "ciclo max" informado via Bluetooth ao fim de cada volta: T
// deve ficar logo acima do maior ciclo medido.
#define CONTROL_LOOP_PERIOD_MS (1)

// IMU LSM6DSR via I2C. A CONFERIR no esquemático: GPIOs de SDA e SCL.
// O endereço (0x6A ou 0x6B, conforme o pino SA0) é detectado automaticamente.
#define GPIO_IMU_SDA                 (35)
#define GPIO_IMU_SCL                 (36)
#define IMU_I2C_FREQ_HZ              (400000)
// +1.0F se ω positivo corresponder a giro anti-horário visto de cima; -1.0F
// se a IMU estiver montada invertida. A CONFERIR girando o robô à mão.
#define IMU_GYRO_Z_SIGN              (1.0F)
// Medição do bias do giroscópio na calibração (robô parado):
// GYRO_BIAS_SAMPLES leituras, uma a cada GYRO_BIAS_SAMPLE_INTERVAL_MS.
#define GYRO_BIAS_SAMPLES            (500)
#define GYRO_BIAS_SAMPLE_INTERVAL_MS (2)

// Marcações laterais. As posições abaixo são índices do vetor de leituras
// laterais, na ordem de GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX ({15, 14, 6, 7}).
// A CONFERIR no robô qual lado é qual.
#define SIDE_SENSORS_LEFT_POSITIONS  {0, 1}
#define SIDE_SENSORS_RIGHT_POSITIONS {2, 3}
// Leitura calibrada (0 a 1000, branco = valor baixo) abaixo da qual o sensor
// está vendo a marcação.
#define MARK_SENSOR_THRESHOLD        (500)
// Pulsos mais curtos que isto são ignorados (ruído), em microssegundos.
#define MARK_MIN_PULSE_US            (1000)

#define MOTOR_MAPPING_PWM (10)
#define VACUUM_BASE_PWM   (100)

#define MAX_MOTOR_PWM (66)

/// Minimum time between recorded map points while mapping, in milliseconds.
#define MAP_POINT_SAVE_INTERVAL       (250)
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
