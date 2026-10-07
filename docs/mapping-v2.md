# Mapeamento v2: modelo cinemático, IMU e marcações

Substitui o mapeamento por derivada dos encoders (`mapping-v1.md`).

## O que o robô faz

- **Calibração (na inicialização):** primeiro mede o bias do giroscópio com o robô
  parado (LED 0 em ciano, mensagem no Bluetooth), depois calibra os sensores IR
  com o robô sendo passado sobre a linha (LED 0 em amarelo).
- **A cada ciclo do mapeamento:** lê os sensores e a IMU, segue a linha com o PD,
  atualiza a pose e detecta as marcações.
  - `v = (v_d + v_e) / 2`, com `v_x = 2πr·N_x / (N_res·Δt)` pelos pulsos do intervalo.
  - `ω = ω_z − b`, pelo giroscópio da IMU.
  - `x += v·Δt·cos θ`, `y += v·Δt·sin θ`, `θ += ω·Δt`.
- **Marcações (borda de descida):** quando um lado deixa de ver a marcação, ela
  conta se o outro lado não ficou ativo durante o pulso (senão é cruzamento).
  Esquerda incrementa `sector`; direita incrementa `rightMarks`.
- **Fim:** quando `rightMarks` chega a 2 (marcação de chegada), o robô para e
  grava o ponto final. O `pause` continua interrompendo a qualquer momento.
- **Laço com período fixo** `CONTROL_LOOP_PERIOD_MS` (`vTaskDelayUntil`). Ao fim
  de cada volta, o robô envia pelo Bluetooth o maior tempo de ciclo medido
  ("Ciclo max"), para ajustar o período.

## Formato do ponto (`map_get` e `map_add`)

Cada item da lista tem 11 campos, nesta ordem:

| # | Campo | Unidade |
| --- | --- | --- |
| 1 | `t` | ms desde a largada |
| 2 | `encoder_left` | pulsos acumulados |
| 3 | `encoder_right` | pulsos acumulados |
| 4 | `v` | mm/s |
| 5 | `omega` | rad/s |
| 6 | `x` | mm |
| 7 | `y` | mm |
| 8 | `theta` | rad |
| 9 | `sector` | marcações esquerdas válidas |
| 10 | `right_marks` | marcações direitas válidas |
| 11 | `speed` | % de PWM |

O formato antigo do `map_data.dat` é incompatível: rode `map_clear_storage`
antes do primeiro teste.

## A conferir no `env.hpp` antes do teste

- `GPIO_IMU_SDA` e `GPIO_IMU_SCL` (hoje `-1`: sem eles, ω é gravado como 0).
- `IMU_GYRO_Z_SIGN`: girando o robô no sentido anti-horário (visto de cima), θ
  deve aumentar.
- `SIDE_SENSORS_LEFT_POSITIONS` e `SIDE_SENSORS_RIGHT_POSITIONS`.
- `MARK_SENSOR_THRESHOLD` e `MARK_MIN_PULSE_US`.
- `WHEEL_RADIUS` e `ENCODER_PULSES_PER_ROTATION`.
- `CONTROL_LOOP_PERIOD_MS`, a partir do "Ciclo max".
