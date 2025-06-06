#pragma once

#include "LiDARConfig.hpp"
#include <driver/uart.h>
#include <esp_err.h>
#include <span>
#include <cstddef> // std::size_t, std::byte

namespace lidar
{

  class [[nodiscard]] UART_HAL final
  {
  public:
    UART_HAL() = default;
    ~UART_HAL();

    esp_err_t init(const LiDARConfig &cfg);
    esp_err_t read(std::span<std::byte> buffer, std::size_t &bytesRead, uint32_t timeoutMs = 20) const;
    void deinit();

  private:
    uart_port_t uartPort_ = UART_NUM_1;
  };

} // namespace lidar
