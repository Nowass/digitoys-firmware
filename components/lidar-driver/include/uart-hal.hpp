/**
 * @file uart-hal.hpp
 * @brief UART helper class used by the LiDAR driver.
 */
#pragma once

#include "LiDARConfig.hpp"
#include <driver/uart.h>
#include <esp_err.h>
#include <span>
#include <cstddef> // std::size_t, std::byte

namespace lidar
{

  /// Thin wrapper around the ESP-IDF UART driver.
  class [[nodiscard]] UART_HAL final
  {
  public:
    UART_HAL() = default;
    ~UART_HAL();

    /// Initialize UART peripheral
    /// @param cfg configuration with port and pins
    esp_err_t init(const LiDARConfig &cfg);
    /// Read bytes with timeout
    /// @param buffer destination buffer
    /// @param bytesRead number of bytes actually read
    /// @param timeoutMs timeout in milliseconds
    esp_err_t read(std::span<std::byte> buffer, std::size_t &bytesRead, uint32_t timeoutMs = 20) const;
    /// Deinitialize UART port
    void deinit();

  private:
    uart_port_t uartPort_ = UART_NUM_1;
  };

} // namespace lidar
