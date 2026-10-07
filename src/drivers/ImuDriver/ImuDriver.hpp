#ifndef IMU_DRIVER_HPP
#define IMU_DRIVER_HPP

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/// Pinos e frequência do barramento I2C da IMU.
struct ImuPins {
  int      gpioSda;
  int      gpioScl;
  uint32_t frequencyHz;
};

/// Driver mínimo do LSM6DSR (ST) por I2C: só o giroscópio, eixo z.
/// O endereço (0x6A ou 0x6B, conforme o pino SA0) é detectado lendo o
/// registrador WHO_AM_I nos dois endereços.
class ImuDriver {
public:
  explicit ImuDriver(ImuPins pins);
  ~ImuDriver();

  /// true se o chip foi encontrado e configurado.
  bool    isReady() const { return ready_; }
  uint8_t address() const { return address_; }

  /// Velocidade angular no eixo z, em rad/s (sem bias descontado).
  bool readGyroZ(float *radPerSec);

private:
  static constexpr uint8_t kAddressLow  = 0x6A; // SA0 em nível baixo
  static constexpr uint8_t kAddressHigh = 0x6B; // SA0 em nível alto
  static constexpr uint8_t kRegWhoAmI   = 0x0F;
  static constexpr uint8_t kWhoAmIValue = 0x6B;
  static constexpr uint8_t kRegCtrl2G   = 0x11;
  static constexpr uint8_t kRegCtrl3C   = 0x12;
  static constexpr uint8_t kRegOutZLowG = 0x26;
  static constexpr int     kTimeoutMs   = 5;

  // CTRL2_G: ODR do giroscópio 1,66 kHz (1000b) e fundo de escala ±2000 dps
  // (FS_G = 11b). ±2000 dps cobre ~34,9 rad/s.
  static constexpr uint8_t kCtrl2GValue = 0x8C;
  // Sensibilidade a ±2000 dps: 70 mdps/LSB, convertida para rad/s.
  static constexpr float kRadPerSecPerLsb = 0.070F * 3.14159265F / 180.0F;
  // CTRL3_C: BDU = 1 (bit 6) e IF_INC = 1 (bit 2).
  static constexpr uint8_t kCtrl3CValue   = 0x44;
  static constexpr uint8_t kCtrl3CSwReset = 0x01;

  const char             *tag_     = "ImuDriver";
  i2c_master_bus_handle_t bus_     = nullptr;
  i2c_master_dev_handle_t dev_     = nullptr;
  uint8_t                 address_ = 0;
  bool                    ready_   = false;

  bool attachAt(uint8_t address, uint32_t frequencyHz);
  bool readRegisters(uint8_t reg, uint8_t *data, size_t length);
  bool writeRegister(uint8_t reg, uint8_t value);
};

inline ImuDriver::ImuDriver(ImuPins pins) {
  if(pins.gpioSda < 0 || pins.gpioScl < 0) {
    ESP_LOGE(tag_, "Pinos SDA/SCL da IMU nao definidos (env.hpp)");
    return;
  }

  i2c_master_bus_config_t busConfig = {};
  busConfig.i2c_port                = -1; // escolhe uma porta livre
  busConfig.sda_io_num              = static_cast<gpio_num_t>(pins.gpioSda);
  busConfig.scl_io_num              = static_cast<gpio_num_t>(pins.gpioScl);
  busConfig.clk_source              = I2C_CLK_SRC_DEFAULT;
  busConfig.glitch_ignore_cnt       = 7;
  busConfig.flags.enable_internal_pullup = true;
  if(i2c_new_master_bus(&busConfig, &bus_) != ESP_OK) {
    ESP_LOGE(tag_, "Falha ao criar o barramento I2C");
    bus_ = nullptr;
    return;
  }

  if(!attachAt(kAddressLow, pins.frequencyHz) &&
     !attachAt(kAddressHigh, pins.frequencyHz)) {
    ESP_LOGE(tag_, "LSM6DSR nao encontrado em 0x6A nem 0x6B");
    return;
  }

  // Reset por software e configuração do giroscópio.
  if(!writeRegister(kRegCtrl3C, kCtrl3CSwReset)) {
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  if(!writeRegister(kRegCtrl3C, kCtrl3CValue) ||
     !writeRegister(kRegCtrl2G, kCtrl2GValue)) {
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(20)); // tempo de partida do giroscópio

  ready_ = true;
  ESP_LOGI(tag_, "LSM6DSR pronto no endereco 0x%02X", address_);
}

inline ImuDriver::~ImuDriver() {
  if(dev_ != nullptr) {
    i2c_master_bus_rm_device(dev_);
  }
  if(bus_ != nullptr) {
    i2c_del_master_bus(bus_);
  }
}

inline bool ImuDriver::attachAt(uint8_t address, uint32_t frequencyHz) {
  if(i2c_master_probe(bus_, address, kTimeoutMs) != ESP_OK) {
    return false;
  }

  i2c_device_config_t devConfig = {};
  devConfig.dev_addr_length     = I2C_ADDR_BIT_LEN_7;
  devConfig.device_address      = address;
  devConfig.scl_speed_hz        = frequencyHz;
  if(i2c_master_bus_add_device(bus_, &devConfig, &dev_) != ESP_OK) {
    dev_ = nullptr;
    return false;
  }

  uint8_t whoAmI = 0;
  if(!readRegisters(kRegWhoAmI, &whoAmI, 1) || whoAmI != kWhoAmIValue) {
    ESP_LOGW(tag_, "0x%02X respondeu WHO_AM_I = 0x%02X", address, whoAmI);
    i2c_master_bus_rm_device(dev_);
    dev_ = nullptr;
    return false;
  }

  address_ = address;
  return true;
}

inline bool ImuDriver::readRegisters(uint8_t reg, uint8_t *data,
                                     size_t length) {
  return i2c_master_transmit_receive(dev_, &reg, 1, data, length, kTimeoutMs) ==
         ESP_OK;
}

inline bool ImuDriver::writeRegister(uint8_t reg, uint8_t value) {
  const uint8_t buffer[2] = {reg, value};
  return i2c_master_transmit(dev_, buffer, sizeof(buffer), kTimeoutMs) ==
         ESP_OK;
}

inline bool ImuDriver::readGyroZ(float *radPerSec) {
  if(!ready_ || radPerSec == nullptr) {
    return false;
  }
  uint8_t raw[2] = {0, 0};
  if(!readRegisters(kRegOutZLowG, raw, sizeof(raw))) {
    return false;
  }
  const int16_t value =
      static_cast<int16_t>(static_cast<uint16_t>(raw[1]) << 8 | raw[0]);
  *radPerSec = static_cast<float>(value) * kRadPerSecPerLsb;
  return true;
}

#endif // IMU_DRIVER_HPP
