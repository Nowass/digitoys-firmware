#pragma once
#include "esp_err.h"
#include "LiDARConfig.hpp"

namespace lidar {

class UART_HAL {
public:
  /**
   * @brief Initialize UART in DMA mode using parameters from cfg.
   */
  esp_err_t init(const LiDARConfig &cfg);

  /**
   * @brief Send raw bytes over UART.
   */
  esp_err_t write(const uint8_t *data, size_t len);

  /**
   * @brief Read up to len bytes from the DMA buffer.
   * @param buf      Destination buffer
   * @param len      Maximum number of bytes to read
   * @param out_len  Actual number of bytes read
   */
  esp_err_t read(uint8_t *buf, size_t len, size_t &out_len);

private:
  uart_port_t _uartPort;   // << store the port here
};

} // namespace lidar