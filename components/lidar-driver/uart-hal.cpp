#include "uart-hal.hpp"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static constexpr const char *TAG = "UART_HAL";

namespace lidar
{

    esp_err_t UART_HAL::init(const LiDARConfig &cfg)
    {
        _uartPort = cfg.uartPort;

        uart_config_t uart_cfg = {}; // Zero-initialize everything (including anonymous fields)

        uart_cfg.baud_rate = 230400;
        uart_cfg.data_bits = UART_DATA_8_BITS;
        uart_cfg.parity = UART_PARITY_DISABLE;
        uart_cfg.stop_bits = UART_STOP_BITS_1;
        uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

        esp_err_t err = uart_param_config(cfg.uartPort, &uart_cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "uart_param_config failed");
            return err;
        }

        err = uart_set_pin(cfg.uartPort, cfg.txPin, cfg.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "uart_set_pin failed");
            return err;
        }

        // install driver with RX DMA buffer
        err = uart_driver_install(cfg.uartPort,
                                  cfg.dmaBufferLen * 2, // RX buffer size
                                  0,                    // TX buffer size
                                  0,                    // event queue size
                                  nullptr,
                                  0);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "uart_driver_install failed");
        }
        return err;
    }

    esp_err_t UART_HAL::write(const uint8_t *data, size_t len)
    {
        int written = uart_write_bytes(_uartPort,
                                       (const char *)data, len);
        return written == (int)len ? ESP_OK : ESP_FAIL;
    }

    esp_err_t UART_HAL::read(uint8_t *buf, size_t len, size_t &out_len)
    {
        int r = uart_read_bytes(_uartPort, buf, len, pdMS_TO_TICKS(20));
        if (r <= 0)
        {
            out_len = 0;
            return ESP_FAIL;
        }
        out_len = static_cast<size_t>(r);
        return ESP_OK;
    }

} // namespace lidar