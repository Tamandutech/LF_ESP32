# Mapeamento da Pista V1

O mapemaneto da pista pode ser feito com os dados dos encoders, de uma IMU, ou da fusão de ambos.

O importante é manter o contrato de qual medida será salva no arquivo de mapeamento.

Nesse caso, vamos salvar a contagem dos encoders de cada roda e o ângulo de curvatura da linha.

## Formato do arquivo de mapeamento

```csv
index,encoder_left,encoder_right,encoder_derivative,encoder_derivative_average,speed,point_type
```
## Estrutura de dados do mapa

> A estrutura MapPoint não armazena o index pois o índice é do próprio array onde este ponto está armazenado.
> O índice é explicitamente usado durante a comunicação bluetooth e para salvar no arquivo na memória flash.


```cpp
struct MapPoint {
  int32_t encoder_left;
  int32_t encoder_right;
  float encoder_derivative;
  float encoder_derivative_average;
  float speed;
  enum PointType {
    AUTO_MARK, // Marked by the robot automatically
    MANUAL_MARK, // Added by the user manually via CLI
    STOP_COMMAND_MARK, // Added when the user sends the stop command via CLI during mapping mode (only for the last point)
    UNKNOWN_MARK, // Unknown mark type
    CURVE_START_MARK, // Derivative above the moving average (start of a curve)
    CURVE_END_MARK // Derivative below the moving average (end of a curve)
  } point_type;
}
```

## Algoritmo de mapeamento

### 1. Entrar no modo de mapeamento

Quando o estado do robô se torna "MAPPING", o robô deve entrar no modo de mapeamento.

Limpa o array de pontos do mapa na memória RAM.


### 2. Simular o calculo do angulo de curvatura da linha usando apenas a diferença de contagem dos encoders de cada roda

> Não leva em consideração a distância entre as rodas, portanto não usa unidades de medida padrão. Apenas considera a diferença de contagem para calcular a intensidade proporcional da curvatura.

O ângulo de curvatura da linha é calculado a partir da diferença de contagem dos encoders de cada roda.

delta_encoder = encoder_right - encoder_left

### 3. Calcular a derivada de delta_encoder em relação ao tempo

Calcular a derivada de delta_encoder em relação ao tempo:
```cpp
delta_encoder = encoder_right - encoder_left
delta_time = time - last_time
encoder_derivative = (delta_encoder - last_delta_encoder) / delta_time
last_delta_encoder = delta_encoder
last_time = time
```

### 4. Calcular média móvel das derivadas de delta_encoder em relação ao tempo

Calcular a média móvel das derivadas de delta_encoder em relação ao tempo.

Média móvel com a biblioteca: https://github.com/AlexandreHiroyuki/DataTome

### 5. Se a derivada atual for diferente da média móvel das derivadas (por uma margem de erro), salvar o ponto no mapa

Se a derivada atual for maior que a média móvel das derivadas (por uma margem de erro), salvar o ponto no mapa como inicio de curva.

Se a derivada atual for menor que a média móvel das derivadas (por uma margem de erro), salvar o ponto no mapa como fim de curva.

### 6. Salvar ponto no mapa em memória RAM

Salvar o ponto no mapa em memória RAM (ou seja, numa array de structs de pontos do tipo `MapPoint`).

Um novo ponto deve ser salvo a cada `MAP_POINT_SAVE_PULSE_INTERVAL` pulsos de encoder (média das duas rodas), configurado pela CLI via comunicação bluetooth.

Ao parar o robô no modo de mapeamento, o robô também deve salvar o ponto onde parou no mapa independente do intervalo de pulsos configurado.

### 7. Salvar ponto no mapa em memória Flash somente com a confirmação do usuário via comunicação bluetooth

Salvar o mapa em memória Flash somente com a confirmação do usuário via comunicação bluetooth. Quando o usuário enviar o comando, o robô deve salvar o mapa num arquivo na memória Flash.

## Lendo mapeamento

### 1. Ao iniciar o robô, ler o mapa em memória Flash

Ler o mapa em memória Flash e salvar na memória RAM.

### 2. No modo de corrida, ler o mapa em memória RAM

Ler o mapa em memória RAM e usar a velocidade do ponto atual como velocidade base para o PID de controle da velocidade do motor.

### 3. Parar a corrida

Após o último ponto do mapa, o robô deve parar.
